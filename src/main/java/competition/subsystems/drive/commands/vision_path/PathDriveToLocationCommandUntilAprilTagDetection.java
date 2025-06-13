package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
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

    @Inject
    public PathDriveToLocationCommandUntilAprilTagDetection(
            BaseSwerveDriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
            HeadingModule.HeadingModuleFactory headingModuleFactory,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
            RobotAssertionManager robotAssertionManager,
            CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager, coprocessorCommunicationSubsystem);
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public PathDriveToLocationCommandUntilAprilTagDetection setTarget(
            int aprilTagID) {
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