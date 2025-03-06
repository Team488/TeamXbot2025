package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class PathDriveToLocationUntilAprilTagDetectionDynamic
        extends PathDriveToLocation {
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    @Inject
    public PathDriveToLocationUntilAprilTagDetectionDynamic(
            BaseSwerveDriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
            HeadingModule.HeadingModuleFactory headingModuleFactory,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
            RobotAssertionManager robotAssertionManager,
            CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, aprilTagVisionSubsystem,
                robotAssertionManager, coprocessorCommunicationSubsystem);
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public PathDriveToLocationUntilAprilTagDetectionDynamic setTarget(
            Pose2d target) {
        super.setTarget(target);
        return this;
    }

    @Override
    public boolean isFinished() {
        return (curves != null && curves.hasAlignToReefAprilTagOptions()
                ? aprilTagVisionSubsystem
                .doesCameraBestObservationHaveAprilTagId(
                        curves.getAlignToReefAprilTagOptions()
                                .getCamera()
                                .equals(
                                        XTableValues.AprilTagCamera.FRONT_LEFT)
                                ? Cameras.FRONT_LEFT_CAMERA.getIndex()
                                : Cameras.FRONT_RIGHT_CAMERA.getIndex(),
                        curves.getAlignToReefAprilTagOptions()
                                .getAprilTagID())
                : curves != null)
                || logic.recommendIsFinished(
                pose.getCurrentPose2d(), drive.getPositionalPid(), headingModule)
                || super.isFinished();
    }
}
