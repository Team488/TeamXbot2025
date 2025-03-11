package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.kobe.xbot.JClient.CachedSubscriber;
import org.kobe.xbot.JClient.XTablesClient;

import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.math.PIDManager.PIDManagerFactory;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

/**
 * Command to align the robot with a creeper target using vision-based error
 * measurements. <p> This command retrieves error values (in pixels) from the
 * vision system via the coprocessor, normalizes the error relative to the
 * camera resolution, applies a gain factor to scale the drive power
 * appropriately, and then moves the robot to align with the target.
 * </p>
 */
public class AlignWithCreeperCommand extends BaseCommand {
    private boolean forceStop = false;
    private final PIDManager pidManager;
    
    final DriveSubsystem drive;
    final CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;

    private final String tableLeftDistance = "verticalEdgeLeftDistancePx";
    private final String tableRightDistance = "verticalEdgeRightDistancePx";

    private final StringProperty photonVisionFrontLeftHostname;
    private final StringProperty photonVisionFrontRightHostname;

    private final DoubleProperty photonVisionFrontLeftResX;
    private final DoubleProperty photonVisionFrontRightResX;
    private final DoubleProperty driveGain;
    private final DoubleProperty logScalar;
    private final DoubleProperty logErrScalar;
    private final DoubleProperty maxError;
    private final DoubleProperty errorThresholdPixels;

    private CachedSubscriber leftOffsetPixelsSubscriber;
    private CachedSubscriber rightOffsetPixelsSubscriber;


    private String hostname;
    private Cameras camera;
    private int resolution;

    protected Boolean isCenteredConfidently = false;
    /**
     * Constructs a new AlignWithCreeperCommand.
     *
     * @param vision                    the vision subsystem used for target
     *                                  detection.
     * @param drive                     the drive subsystem for robot movement.
     * @param coprocessorCommunications subsystem for communicating with the
     *                                  coprocessor.
     * @param electricalContract        the electrical contract.
     * @param pose                      the pose subsystem.
     * @param headingModuleFactory      factory to create heading modules.
     * @param reefCoordinateGenerator   the coordinate generator.
     * @param pf                        the property factory for persistent
     *                                  properties.
     */
    @Inject
    public AlignWithCreeperCommand(AprilTagVisionSubsystemExtended vision,
                                   DriveSubsystem drive,
                                   CoprocessorCommunicationSubsystem coprocessorCommunications,
                                   ElectricalContract electricalContract, PoseSubsystem pose,
                                   HeadingModule.HeadingModuleFactory headingModuleFactory,
                                   ReefCoordinateGenerator reefCoordinateGenerator,
                                   PropertyFactory pf,
                                   PIDManagerFactory pidFactory) {
        this.addRequirements(drive);
        this.drive = drive;
        this.pidManager = pidFactory.create("AlignWithCreeperCommand", 0.6, 0.0001, 0.9);
        this.coprocessorCommunicationSubsystem = coprocessorCommunications;
        pf.setPrefix("AlignWithCreeperCommand/");
        this.driveGain = pf.createPersistentProperty("Drive Gain", 0.18);
        this.logScalar = pf.createPersistentProperty("Log Scalar", 15);
        this.logErrScalar = pf.createPersistentProperty("Log input Err Scalar", 9);
        this.maxError = pf.createPersistentProperty("Max Error", 0.2);
        this.photonVisionFrontLeftHostname = pf.createPersistentProperty(
                "Photon Vision Front Left Hostname", "photonvisionfrontleft");
        this.photonVisionFrontRightHostname = pf.createPersistentProperty(
                "Photon Vision Front Right Hostname", "photonvisionfrontright");
        this.photonVisionFrontLeftResX = pf.createPersistentProperty(
                "Photon Vision Front Left Resoulution X", 800);
        this.photonVisionFrontRightResX = pf.createPersistentProperty(
                "Photon Vision Front Right Resoulution X", 800);
        this.errorThresholdPixels = pf.createPersistentProperty(
                "Pixel error Threshold", 35);
    }

    /**
     * Initializes the command.
     * <p>
     * This method attempts to obtain an XTablesClient from the coprocessor
     * communication subsystem. If the client is null or the camera value is
     * unknown, the command is canceled. It then sets up subscribers for the left
     * distance, right distance, and alignment confidence.
     * </p>
     */
    @Override
    public void initialize() {
        forceStop = false;
        isCenteredConfidently = false;
        XTablesClient client =
                this.coprocessorCommunicationSubsystem.tryGetXTablesClient();
        if (client == null) {
            log.warn("Failed to obtain a valid XTablesClient from CoprocessorCommunicationSubsystem. "
                    + "Aborting command initialization.");
            cancel();
            return;
        }

        // Determine active camera and retrieve its corresponding resolution and hostname.
        if (camera.equals(Cameras.FRONT_LEFT_CAMERA)) {
            this.resolution = (int) photonVisionFrontLeftResX.get();
            this.hostname = photonVisionFrontLeftHostname.get();
        } else if (camera.equals(Cameras.FRONT_RIGHT_CAMERA)) {
            this.resolution = (int) photonVisionFrontRightResX.get();
            this.hostname = photonVisionFrontRightHostname.get();
        } else {
            log.warn("Encountered an unrecognized camera value. Aborting command to avoid unintended drive behavior.");
            cancel();
            return;
        }

        if(this.leftOffsetPixelsSubscriber == null){
            this.leftOffsetPixelsSubscriber = new CachedSubscriber(hostname + "." + tableLeftDistance, client,5);
        }

        if(this.rightOffsetPixelsSubscriber == null){
            this.rightOffsetPixelsSubscriber = new CachedSubscriber(hostname + "." + tableRightDistance, client,5);
        }
    }

    private boolean isDriveStopped(){
        return drive.getActiveSwerveModuleSubsystem().getCurrentState().equals(new SwerveModuleState(0, Rotation2d.kZero));
    }

    /**
     * Executes the alignment logic.
     * <p>
     * This method retrieves error values from the vision system via subscribed
     * channels. It computes a normalized error relative to the camera resolution,
     * applies a gain factor to scale the drive power, clamps the result to the
     * range [-1, 1], and adjusts the direction based on which side (left or
     * right) the error is measured. Finally, it commands the drive subsystem to
     * move accordingly.
     * </p>
     */
    @Override
    public void execute() {
        super.execute();

        if(forceStop){
            log.info("FORCE STOP");
            drive.stop();
            return;
        }
        

        Integer leftDistance = this.leftOffsetPixelsSubscriber.getAsInteger(null);
        Integer rightDistance = this.rightOffsetPixelsSubscriber.getAsInteger(null);

        if (leftDistance == null || rightDistance == null) {
            // since subscriber is not updating fast enough, sometimes we get nulls
            aKitLog.record("Subcriber updated?", false);
            log.warn("Vision offsets are returning null! Is alignment up?");
            return;
        }
        else{
            aKitLog.record("Subcriber updated?", true);
        }

        if (leftDistance == -1 && rightDistance == -1) {
            log.warn("No valid left or right distance measurements received from the camera.");
            forceStop = true;
            return;
        }


        if(leftDistance == -1 || rightDistance == -1){
            this.isCenteredConfidently = false;
        }
        else{
            this.isCenteredConfidently = Math.abs(leftDistance-rightDistance) < this.errorThresholdPixels.get();
        }


        aKitLog.record("Is Centered confidently", this.isCenteredConfidently);
        aKitLog.record("Left Distance Pixels", leftDistance);
        aKitLog.record("Right Distance Pixels", rightDistance);

        double error;
        if (isCenteredConfidently) {
            // Alignment is achieved; stop any drive movement.
            log.info("Vision is saying we are centered confidently!");
            // If the error is within the acceptable range, act as a "deadband"
            error = 0;
        }
        else{
            // Determine which side the misalignment error should be taken from.
            boolean offToTheLeft;

            if (leftDistance == -1) {
                offToTheLeft = false; // we dont see left edge, means right side is "too" visible eg off to the right
            } else if (rightDistance == -1) {
                offToTheLeft = true; // we dont see right  edge, means left side is "too" visible eg off to the left
            } else {
                // Choose the pixel error value from the side indicating misalignment.
                offToTheLeft = leftDistance < rightDistance; // we see both edges, so the one that is "closer to the center" is the one we are off by
            }

            aKitLog.record("Off to the left?: ",offToTheLeft);

            // Calculate the error as a function of the left and right distance from the center.
            error = costFunc(leftDistance, rightDistance);

            // add sign
            // if we are off to the right (!offtotheleft) and we want to minimize our error in the right direction,
            // we need to invert
            if (!offToTheLeft) {
                error = -error;
            }


        }

        aKitLog.record("creeper error from cost function: ",error);

        // power.
        double drivePower = driveGain.get() * pidManager.calculate(0, error);
        aKitLog.record("Creeper Drive Power", drivePower);

        // Create the drive command vector (Only drive power along the Y-axis,
        // side-to-side).
        XYPair pair = new XYPair(0, drivePower);
        this.drive.drive(pair, 0.0, true);

    }

    // scaled cost function
    private double costFunc(int leftErrPX, int rightErrPX){
        if(leftErrPX == -1 || rightErrPX == -1){
            // much too off aligned
            return maxError.get();
        }
        // abs error
        double err = (double) Math.abs(leftErrPX-rightErrPX);
        // log is negative when the input is less that 1, 
        // to make this never happen, add 1
        double errFunc = Math.log(err/logErrScalar.get()+1)/logScalar.get(); 
        return Math.max(-maxError.get(), Math.min(errFunc, maxError.get()));
    }

    /**
     * Checks if the alignment is completed.
     *
     * @return true if the vision system indicates the robot is confidently
     * aligned, false otherwise.
     */
    @Override
    public boolean isFinished() {
        // us being centered is only valuable if we arent currently moving
        return forceStop && this.isDriveStopped() && (this.isCenteredConfidently == null || this.isCenteredConfidently);
    }

    /**
     * Ends the command.
     * <p>
     * This method is called when the command finishes either normally or due to
     * interruption. It stops the drive subsystem and logs a warning if the
     * command was interrupted.
     * </p>
     *
     * @param interrupted true if the command was interrupted or canceled.
     */
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            log.warn("Alignment command interrupted before completion.");
        }
        drive.stop();
    }

    /**
     * Returns the currently selected camera used for alignment.
     *
     * @return the camera currently in use.
     */
    public Cameras getCamera() {
        return camera;
    }

    /**
     * Sets the camera to be used for alignment.
     *
     * @param camera the camera to set.
     * @return this instance of AlignWithCreeperCommand, allowing for method
     * chaining.
     */
    public AlignWithCreeperCommand setCamera(Cameras camera) {
        this.camera = camera;
        return this;
    }
}
