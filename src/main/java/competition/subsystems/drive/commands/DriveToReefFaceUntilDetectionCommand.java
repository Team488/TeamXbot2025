package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;

public class DriveToReefFaceUntilDetectionCommand extends SwerveSimpleTrajectoryCommand {

    Pose2d targetReefFacePose;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;

    @Inject
    public DriveToReefFaceUntilDetectionCommand(DriveSubsystem drive, PoseSubsystem pose,
                                                PropertyFactory pf,
                                                HeadingModule.HeadingModuleFactory headingModuleFactory,
                                                AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
                                                RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    // TODO: uncomment
    public void setTargetReefFacePose(Landmarks.ReefFace targetReefFace) {
//        this.targetReefFacePose = Landmarks.getReefFacePose(targetReefFace);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();
        swervePoints.add(XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(targetReefFacePose, 10));
        this.logic.setKeyPoints(swervePoints);
        this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return aprilTagVisionSubsystem.reefAprilTagCameraHasCorrectTarget(
                aprilTagVisionSubsystem.getTargetAprilTagID(targetReefFacePose))
                || logic.recommendIsFinished(pose.getCurrentPose2d(), drive.getPositionalPid(), headingModule);
    }
}
