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
        Pose2d leftStationInterstitialPoint = new Pose2d(
                Landmarks.BlueFarLeftBranchB.getX() - 0.9144, // TODO: Tune
                Landmarks.BlueFarLeftBranchB.getY() + 0.9144,
                Landmarks.BlueFarLeftBranchB.getRotation());
        Pose2d rightStationInterstitialPoint = new Pose2d(
                Landmarks.BlueFarRightBranchA.getX() - 0.9144,
                Landmarks.BlueFarRightBranchA.getY() - 0.9144,
                Landmarks.BlueFarLeftBranchA.getRotation());

        if (station == Landmarks.CoralStation.LEFT) {
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(leftStationInterstitialPoint), 10));
        }
        else {
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(rightStationInterstitialPoint), 10));
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
