package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import javax.inject.Inject;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

public class PathDriveToLocationUntilAprilTagDetection
        extends PathDriveToLocation {
    int aprilTagID;

    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    private Cameras camera = Cameras.FRONT_LEFT_CAMERA;

    @Inject
    public PathDriveToLocationUntilAprilTagDetection(
            BaseSwerveDriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
            HeadingModule.HeadingModuleFactory headingModuleFactory,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
            RobotAssertionManager robotAssertionManager,
            CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, aprilTagVisionSubsystem,
                robotAssertionManager, coprocessorCommunicationSubsystem);
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public PathDriveToLocationUntilAprilTagDetection setTarget(
            Pose2d target, int aprilTagID) {
        this.setTarget(target);
        this.aprilTagID = aprilTagID;
        return this;
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