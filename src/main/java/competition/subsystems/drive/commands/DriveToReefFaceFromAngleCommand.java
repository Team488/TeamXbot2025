package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;

public class DriveToReefFaceFromAngleCommand extends SwerveSimpleTrajectoryCommand {

    PoseSubsystem pose;
    Pose2d targetReefFacePose;
    boolean kinematics = true;

    @Inject
    public DriveToReefFaceFromAngleCommand(DriveSubsystem drive, PoseSubsystem pose,
                                           PropertyFactory pf,
                                           HeadingModule.HeadingModuleFactory headingModuleFactory,
                                           RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
        this.pose = pose;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        targetReefFacePose = Landmarks.getReefFacePose(pose.getReefFaceFromAngle());
        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();
        swervePoints.add(new XbotSwervePoint(targetReefFacePose, 10));
        this.logic.setKeyPoints(swervePoints);
        if (kinematics) {
            this.logic.setGlobalKinematicValues(new SwervePointKinematics(2, 0, 0, 4.5));
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        }
        else {
            this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.ConstantVelocity);
        }
        super.initialize();
    }
}
