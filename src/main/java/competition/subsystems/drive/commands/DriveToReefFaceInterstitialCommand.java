package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
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

public class DriveToReefFaceInterstitialCommand extends SwerveSimpleTrajectoryCommand {

    boolean kinematics = true;
    Landmarks.ReefFace reefFace;

    @Inject
    public DriveToReefFaceInterstitialCommand(DriveSubsystem drive, PoseSubsystem pose,
                                                  PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                                  RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
    }

    public void setTargetReefFaceSection(Landmarks.ReefFace reefFace) {
        this.reefFace = reefFace;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();

        // Interstitial points to avoid rotating into the reef when going for coral station alignment
        Pose2d cageTwoInterstitialPoint = new Pose2d(
                Landmarks.BlueCageTwoStartingLine.getX() - 0.9144, // TODO: Tune for better pathing
                Landmarks.BlueCageTwoStartingLine.getY(),
                Landmarks.BlueFarLeftAlgae.getRotation());
        Pose2d cageFiveInterstitialPoint = new Pose2d(
                Landmarks.BlueCageFiveStartingLine.getX() - 0.9144,
                Landmarks.BlueCageFiveStartingLine.getY(),
                Landmarks.BlueFarRightAlgae.getRotation());

        if (reefFace == Landmarks.ReefFace.CLOSE_LEFT) {
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(cageTwoInterstitialPoint), 10));
        }
        else if (reefFace == Landmarks.ReefFace.FAR_RIGHT) {
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(cageFiveInterstitialPoint), 10));
        }
        this.logic.setKeyPoints(swervePoints);

        if (kinematics) {
            // Make sure goalVelocity is non-zero, or else the robot will wait until it's stopped at the interstitial point before continuing
            this.logic.setGlobalKinematicValues(new SwervePointKinematics(2, 0, 2.5, 2.5));
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        }
        else {
            this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.ConstantVelocity);
        }
        super.initialize();
    }
}
