package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
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
    final DriveSubsystem drive;
    final CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;

    private final String tableLeftDistance = "verticalEdgeLeftDistancePx";
    private final String tableRightDistance = "verticalEdgeRightDistancePx";
    private final String tableCenteredConfidently = "verticalAlignedConfidently";

    private final StringProperty photonVisionFrontLeftHostname;
    private final StringProperty photonVisionFrontRightHostname;

    private final DoubleProperty photonVisionFrontLeftResX;
    private final DoubleProperty photonVisionFrontRightResX;
    private final DoubleProperty driveGain;
    private final DoubleProperty errorThresholdPercentage;


    private Cameras camera;
    private int resolution;
    private CachedSubscriber leftDistanceSubscriber;
    private CachedSubscriber rightDistanceSubscriber;
    private CachedSubscriber centeredConfidentlySubscriber;

    protected Boolean isCenteredConfidently;

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
        this.driveGain = pf.createPersistentProperty("Drive Gain", 0.5);
        this.photonVisionFrontLeftHostname = pf.createPersistentProperty(
                "Photon Vision Front Left Hostname", "photonvisionfrontleft");
        this.photonVisionFrontRightHostname = pf.createPersistentProperty(
                "Photon Vision Front Right Hostname", "photonvisionfrontright");
        this.photonVisionFrontLeftResX = pf.createPersistentProperty(
                "Photon Vision Front Left Resoulution X", 800);
        this.photonVisionFrontRightResX = pf.createPersistentProperty(
                "Photon Vision Front Right Resoulution X", 800);
        this.errorThresholdPercentage = pf.createPersistentProperty(
                "Pixel Error Threshold Percentage", 5);
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
        XTablesClient client =
                this.coprocessorCommunicationSubsystem.tryGetXTablesClient();
        if (client == null) {
            log.warn("Failed to obtain a valid XTablesClient from CoprocessorCommunicationSubsystem. "
                    + "Aborting command initialization.");
            cancel();
            return;
        }
        // Determine active camera and retrieve its corresponding resolution and hostname.
        String hostname;
        if (camera.equals(Cameras.FRONT_LEFT_CAMERA)) {
            resolution = (int) photonVisionFrontLeftResX.get();
            hostname = photonVisionFrontLeftHostname.get();
        } else if (camera.equals(Cameras.FRONT_RIGHT_CAMERA)) {
            resolution = (int) photonVisionFrontRightResX.get();
            hostname = photonVisionFrontRightHostname.get();
        } else {
            log.warn("Encountered an unrecognized camera value. Aborting command to avoid unintended drive behavior.");
            cancel();
            return;
        }
        // Subscribe to vision data channels to ensure processing of only fresh data.
        leftDistanceSubscriber =
                client.subscribe(hostname + "." + tableLeftDistance, 1);
        rightDistanceSubscriber =
                client.subscribe(hostname + "." + tableRightDistance, 1);
        centeredConfidentlySubscriber =
                client.subscribe(hostname + "." + tableRightDistance, 1);
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
        XTablesClient client =
                this.coprocessorCommunicationSubsystem.tryGetXTablesClient();
        if (client == null) {
            return;
        }

        try {
            isCenteredConfidently =
                    this.centeredConfidentlySubscriber.getAsBoolean(null);
            Integer leftDistance = this.leftDistanceSubscriber.getAsInteger(null);
            Integer rightDistance = this.rightDistanceSubscriber.getAsInteger(null);

            if (isCenteredConfidently == null) {
                // No new vision data received; do nothing.
                return;
            } else if (isCenteredConfidently) {
                // Alignment is achieved; stop any drive movement.
                drive.stop();
                return;
            }
            // Determine which side the misalignment error should be taken from.
            boolean offToTheLeft;
            if (leftDistance == null && rightDistance == null) {
                log.warn("No valid left or right distance measurements received from the camera.");
                return;
            } else if (leftDistance == null) {
                offToTheLeft = false;
            } else if (rightDistance == null) {
                offToTheLeft = true;
            } else {
                offToTheLeft = leftDistance < rightDistance;
            }
            // Choose the pixel error value from the side indicating misalignment.
            int moveErrorPx = offToTheLeft ? leftDistance : rightDistance;

            // Calculate the normalized error as a fraction of the camera resolution.
            double normalizedError = (double) moveErrorPx / resolution;

            // If the error is within the acceptable range, log a message and exit the adjustment routine.
            double errorThreshold = errorThresholdPercentage.get();
            if (normalizedError <= errorThreshold) {
                log.info("Alignment error {} is within acceptable limits (threshold: {}). No adjustment necessary.",
                        normalizedError, errorThreshold);
                drive.stop();
                return;
            }

            // Scale the normalized error using the drive gain to compute the drive
            // power.
            double drivePower = driveGain.get() * normalizedError;

            // Clamp the drive power to the safe range of [-1, 1].
            drivePower = Math.max(-1.0, Math.min(1.0, drivePower));

            // Reverse the drive power sign if misalignment is to the left to ensure
            // proper movement direction.
            if (offToTheLeft) {
                drivePower = -drivePower;
            }
            // Create the drive command vector (Only drive power along the Y-axis,
            // side-to-side).
            XYPair pair = new XYPair(0, drivePower);
            this.drive.move(pair, 0);

        } catch (Exception e) {
            log.error(e);
        }
    }

    /**
     * Checks if the alignment is completed.
     *
     * @return true if the vision system indicates the robot is confidently
     * aligned, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return this.isCenteredConfidently != null && this.isCenteredConfidently;
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
