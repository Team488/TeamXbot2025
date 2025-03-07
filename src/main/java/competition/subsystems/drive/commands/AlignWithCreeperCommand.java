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
import xbot.common.math.XYPair;
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
    private boolean FORCESTOP = false;
    private int runIter = 0;
    
    final DriveSubsystem drive;
    final CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;

    private final String tableLeftDistance = "verticalEdgeLeftDistancePx";
    private final String tableRightDistance = "verticalEdgeRightDistancePx";
    private final String tableCenteredConfidently = "verticalAlignedConfidently";

    private final StringProperty photonVisionFrontLeftHostname;
    private final StringProperty photonVisionFrontRightHostname;

    private final DoubleProperty photonVisionFrontLeftResX;
    private final DoubleProperty photonVisionFrontRightResX;
    private final DoubleProperty ITERSPERRUN;
    private final DoubleProperty driveGain;
    private final DoubleProperty logNegativityOffset;
    private final DoubleProperty logScalar;
    private final DoubleProperty logErrScalar;
    private final DoubleProperty MAXERR;
    private final DoubleProperty errorThresholdPercentage;

    private CachedSubscriber isCenteredConfidentlySubscriber;
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
                                   PropertyFactory pf) {
        this.addRequirements(drive);
        this.drive = drive;
        this.coprocessorCommunicationSubsystem = coprocessorCommunications;
        pf.setPrefix("AlignWithCreeperCommand/");
        this.driveGain = pf.createPersistentProperty("Drive Gain", 0.3);
        this.ITERSPERRUN = pf.createPersistentProperty("Iters per run", 5);
        this.logNegativityOffset = pf.createPersistentProperty("Log Negativity, lower means more negative closer to zero", 0.3);
        this.logScalar = pf.createPersistentProperty("Log Scalar", 8);
        this.logErrScalar = pf.createPersistentProperty("Log input Err Scalar", 10);
        this.MAXERR = pf.createPersistentProperty("Max Error", 0.3);
        this.photonVisionFrontLeftHostname = pf.createPersistentProperty(
                "Photon Vision Front Left Hostname", "photonvisionfrontleft");
        this.photonVisionFrontRightHostname = pf.createPersistentProperty(
                "Photon Vision Front Right Hostname", "photonvisionfrontright");
        this.photonVisionFrontLeftResX = pf.createPersistentProperty(
                "Photon Vision Front Left Resoulution X", 800);
        this.photonVisionFrontRightResX = pf.createPersistentProperty(
                "Photon Vision Front Right Resoulution X", 800);
        this.errorThresholdPercentage = pf.createPersistentProperty(
                "Pixel error/res Threshold Percentage", 0.01);
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
        FORCESTOP = false;
        runIter = 0;
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

        if(this.isCenteredConfidentlySubscriber == null){
            this.isCenteredConfidentlySubscriber = new CachedSubscriber(hostname + "." + tableCenteredConfidently, client);
        }

        if(this.leftOffsetPixelsSubscriber == null){
            this.leftOffsetPixelsSubscriber = new CachedSubscriber(hostname + "." + tableLeftDistance, client);
        }

        if(this.rightOffsetPixelsSubscriber == null){
            this.rightOffsetPixelsSubscriber = new CachedSubscriber(hostname + "." + tableRightDistance, client);
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

        if(FORCESTOP){
            log.info("FORCE STOP");
            drive.stop();
            return;
        }
        

        
        // hack to add some periodiciy to the command
        runIter++;
        if(runIter < ITERSPERRUN.get()){
            return;
        }
        runIter = 0; // reset

        try {
            this.isCenteredConfidently = this.isCenteredConfidentlySubscriber.getAsBoolean(null);
            Integer leftDistance = this.leftOffsetPixelsSubscriber.getAsInteger(null);
            Integer rightDistance = this.rightOffsetPixelsSubscriber.getAsInteger(null);
            log.info(String.format("Loff %d Roff %d ifConf %b" , leftDistance,rightDistance,isCenteredConfidently));
            
            if (isCenteredConfidently == null) {
                // No new vision data received; do nothing.
                log.warn("Centered confidently is returning null!");
                FORCESTOP = true;
                cancel();
                return;
            }
            // else if (isCenteredConfidently) {
            //     // Alignment is achieved; stop any drive movement.
            //     FORCESTOP = true;
            //     log.info("Vision is saying we are centered confidently!");
            //     return;
            // }
            // Determine which side the misalignment error should be taken from.
            boolean offToTheLeft;
            if (leftDistance == null || rightDistance == null){
                // should never happen, as the vision code will always send at least a -1
                log.warn("Vision offsets are returning null! Is alignment up?");
                FORCESTOP = true;
                cancel();
                return;
            }
            if (leftDistance == -1 && rightDistance == -1) {
                log.warn("No valid left or right distance measurements received from the camera.");
                FORCESTOP = true;
                cancel();
                return;
            } else if (leftDistance == -1) {
                offToTheLeft = false; // we dont see left edge, means right side is "too" visible eg off to the right
            } else if (rightDistance == -1) {
                offToTheLeft = true; // we dont see right  edge, means left side is "too" visible eg off to the left
            } else {
                offToTheLeft = leftDistance < rightDistance; // we see both edges, so the one that is "closer to the center" is the one we are off by
            }
            // Choose the pixel error value from the side indicating misalignment.
            
            // Calculate the error as a function of the left and right distance from the center.
            double error = costFunc(leftDistance, rightDistance);
            log.info("Normalized error: " + error);
            
            // If the error is within the acceptable range, log a message and exit the adjustment routine.
            
            // double errorThresholdPIX = errorThresholdPercentage.get();
            // if (normalizedError <= errorThreshold) {
            //     log.info("Alignment error {} is within acceptable limits (threshold: {}). No adjustment necessary.",
            //             normalizedError, errorThreshold);
            //     FORCESTOP = true;
            //     return;
            // }
            
            // add sign (right is negative)            
            if (offToTheLeft) {
                error = -error;
            }

            

            // Scale the normalized error using the drive gain to compute the drive
            // power.
            double drivePower = driveGain.get() * error;

            // Clamp the drive power to the safe range of [-1, 1].
            drivePower = Math.max(-1, Math.min(1, drivePower));

            
            

            // sim issue where if drive power is below a certain amount, the robot does not move.
            // if(Math.abs(drivePower) < 0.02){
            //     drivePower = Math.copySign(0.02, drivePower);
            // }

            log.info("Driving with power: " + drivePower);
            
            // Create the drive command vector (Only drive power along the Y-axis,
            // side-to-side).
            XYPair pair = new XYPair(0, drivePower);
            this.drive.move(pair, 0);

            // should we always stop the drive after a move command?
            // this.drive.stop();

        } catch (Exception e) {
            log.error(e);
        }
    }

    // scaled cost function
    private double costFunc(int Lerr, int Rerr){
        if(Lerr == -1 || Rerr == -1){
            // much too off aligned
            return MAXERR.get();
        }
        // abs error
        double err = (double) Math.abs(Lerr-Rerr);
        // this error function takes advantage of the fact that log is negative when the input is less that 1, to add some sort of "slowing down when we get close"
        // to make this never happen, set the negativity offset to 1
        double errFunc = Math.log(err/logErrScalar.get()+logNegativityOffset.get())/logScalar.get(); 
        return Math.max(-MAXERR.get(), Math.min(errFunc, MAXERR.get()));
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
        return this.isDriveStopped() && this.isCenteredConfidently;
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
