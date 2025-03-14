package competition.subsystems.drive.commands.vision_path;

import static edu.wpi.first.units.Units.Meters;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import javax.inject.Inject;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

public class PathDriveToReefFaceCommand extends PathDriveToLocationCommand {
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final DriveSubsystem driveSubsystem;
    private final ElectricalContract electricalContract;

    private Pose2d targetPose;

    protected Landmarks.ReefFace reefFace;

    protected Landmarks.Branch branch;
    @Inject
    PathDriveToReefFaceCommand(PoseSubsystem pose, DriveSubsystem drive,
                               CoprocessorCommunicationSubsystem coprocessorComms, PropertyFactory pf,
                               HeadingModule.HeadingModuleFactory headingModuleFactory,
                               RobotAssertionManager assertionManager,
                               ElectricalContract electricalContract,
                               AprilTagFieldLayout aprilTagFieldLayout,
                               AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        super(drive, pose, pf, headingModuleFactory, aprilTagVisionSubsystem,
                assertionManager, coprocessorComms);
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.driveSubsystem = drive;
        this.electricalContract = electricalContract;
    }

    public PathDriveToReefFaceCommand setBranch(Landmarks.Branch branch) {
        this.branch = branch;
        return this;
    }

    public PathDriveToReefFaceCommand setReefFace(Landmarks.ReefFace reefFace) {
        this.reefFace = reefFace;
        return this;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    @Override
    public void initialize() {
        if (this.branch == null || this.reefFace == null) {
            log.error("There is no branch or reef face set! Cancelling command.");
            cancel();
            return;
        }
        targetPose = Landmarks.getReefFaceBranchPose(
                aprilTagFieldLayout, electricalContract, reefFace, branch);
        if (targetPose == null) {
            log.info("Target pose is null, april tag ID not found.");
            cancel();
            return;
        }
        aKitLog.record("finalPose", targetPose);
        this.setTarget(targetPose);
        this.setAdditionalArguments(
                XTableValues.AdditionalArguments.newBuilder()
                        .setAlliance(CoprocessorCommunicationSubsystem.fromAlliance(
                                DriverStation.getAlliance().orElse(
                                        DriverStation.Alliance.Blue)))
                        .build());
        this.setSafeDistance(
                electricalContract.getDiagonalDistanceDifferenceOfRobotRadius());
        this.setOptions(
                XTableValues.TraversalOptions.newBuilder()
                        .setMetersPerSecond(driveSubsystem.getDriveToWaypointsSpeed().get())
                        .setAccelerationMetersPerSecond(
                                driveSubsystem.getMaxAccelerationMetersPerSecondSquared())
                        .setFinalRotationDegrees(targetPose.getRotation().getDegrees())
                        .build());
        super.initialize();
    }
}