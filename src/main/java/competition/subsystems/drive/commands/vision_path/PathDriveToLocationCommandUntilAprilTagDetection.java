package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class PathDriveToLocationCommandUntilAprilTagDetection
        extends PathDriveToLocationCommand {
    int aprilTagID;

    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    private Cameras camera = Cameras.FRONT_LEFT_CAMERA;

    private Landmarks.Branch branch;
    private Landmarks.ReefFace face;
    private ReefCoordinateGenerator reefCoordinateGenerator;


    @Inject
    public PathDriveToLocationCommandUntilAprilTagDetection(
            BaseSwerveDriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
            HeadingModule.HeadingModuleFactory headingModuleFactory,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
            RobotAssertionManager robotAssertionManager,
            CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem,
            ReefCoordinateGenerator reefCoordinateGenerator) {
        super(drive, pose, pf, headingModuleFactory, aprilTagVisionSubsystem,
                robotAssertionManager, coprocessorCommunicationSubsystem);
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.reefCoordinateGenerator = reefCoordinateGenerator;
    }

    public PathDriveToLocationCommandUntilAprilTagDetection setTarget(
            Landmarks.ReefFace reefFace, Landmarks.Branch branch, int aprilTagID) {
        this.face = reefFace;
        this.branch = branch;
        this.aprilTagID = aprilTagID;
        return this;
    }

    @Override
    public void initialize() {
        Pose2d targetPose =
                reefCoordinateGenerator.getPoseRelativeToReefFaceAndBranch(DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue), this.face, this.branch,
                        Units.Meters.of(1.3), Units.Meters.zero());

        this.aKitLog.record("Path_Drive_Goal_Pose", targetPose);

        this.setTarget(targetPose);
        this.setOptions(XTableValues.TraversalOptions.newBuilder()
                        .setFinalRotationDegrees(targetPose.getRotation().getDegrees()).build());
        super.initialize();


    }

    @Override
    public boolean isFinished() {
        return aprilTagVisionSubsystem.doesCameraBestObservationHaveAprilTagId(
                camera.getIndex(), aprilTagID)
                || logic.recommendIsFinished(
                pose.getCurrentPose2d(), drive.getPositionalPid(), headingModule)
                || super.isFinished();
    }

    public void setAprilTagCamera(Cameras camera) {
        this.camera = camera;
    }
}