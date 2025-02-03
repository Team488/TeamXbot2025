package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;


public class DriveToNearestReefFaceUntilDetectionCommand extends SwerveSimpleTrajectoryCommand {
    DriveSubsystem drive;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    Pose2d targetReefFacePose;
    PoseSubsystem pose;

    @Inject
    public DriveToNearestReefFaceUntilDetectionCommand(DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
                                                       HeadingModule.HeadingModuleFactory headingModuleFactory,
                                                       AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        super(drive, pose, pf, headingModuleFactory);
        this.drive = drive;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.pose = pose;
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        targetReefFacePose = pose.getClosestReefFacePose();

        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();
        swervePoints.add(new XbotSwervePoint(targetReefFacePose, 10));
        this.logic.setKeyPoints(swervePoints);
        this.logic.setEnableConstantVelocity(true);
        this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        aKitLog.record("targetReefFacePose", targetReefFacePose);
    }

    @Override
    public boolean isFinished() {
        return aprilTagVisionSubsystem.reefAprilTagCameraHasCorrectTarget(
                aprilTagVisionSubsystem.getTargetAprilTagID(targetReefFacePose))
                || logic.recommendIsFinished(pose.getCurrentPose2d(), drive.getPositionalPid(), headingModule);
    }

}

