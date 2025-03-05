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

public class DriveToCoralStationInterstitialCommand extends SwerveSimpleTrajectoryCommand {

    boolean kinematics = true;
    Landmarks.CoralStation station = Landmarks.CoralStation.LEFT;

    @Inject
    public DriveToCoralStationInterstitialCommand(DriveSubsystem drive, PoseSubsystem pose,
                                                  PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                                  RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
    }

    public void setTargetCoralStationSection(Landmarks.CoralStation station) {
        this.station = station;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();

        // Interstitial points to avoid rotating into the reef when going for coral station alignment
        Pose2d firstLeftStationInterstitialPoint = new Pose2d(
                Landmarks.BlueFarLeftBranchB.getX() - 0.8, // TODO: Tune for better pathing
                Landmarks.BlueFarLeftBranchB.getY() + 1,
                Landmarks.BlueFarLeftBranchB.getRotation());
        Pose2d secondLeftStationInterstitialPoint = new Pose2d(
                firstLeftStationInterstitialPoint.getX() - 0.5,
                firstLeftStationInterstitialPoint.getY(),
                firstLeftStationInterstitialPoint.getRotation()
        );

        Pose2d firstRightStationInterstitialPoint = new Pose2d(
                Landmarks.BlueFarRightBranchA.getX() - 0.8,
                Landmarks.BlueFarRightBranchA.getY() - 1,
                Landmarks.BlueFarRightBranchA.getRotation());
        Pose2d secondRightStationInterstitialPoint = new Pose2d(
                firstRightStationInterstitialPoint.getX() - 0.5,
                firstRightStationInterstitialPoint.getY(),
                firstRightStationInterstitialPoint.getRotation()
        );

        if (station == Landmarks.CoralStation.LEFT) {
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(firstLeftStationInterstitialPoint), 10));
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(secondLeftStationInterstitialPoint), 10));
        }
        else {
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(firstRightStationInterstitialPoint), 10));
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(secondRightStationInterstitialPoint), 10));
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
