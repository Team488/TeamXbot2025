package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;

public class DriveToCoralStationSectionCommand extends SwerveSimpleTrajectoryCommand {

    Pose2d targetCoralStationSection;
    boolean kinematics = true;
    Landmarks.CoralStation station = Landmarks.CoralStation.LEFT;

    @Inject
    public DriveToCoralStationSectionCommand(DriveSubsystem drive, PoseSubsystem pose,
                                             PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                             RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
    }

    public void setTargetCoralStationSection(Landmarks.CoralStation station, Landmarks.CoralStationSection section) {
        this.targetCoralStationSection = Landmarks.getCoralStationSectionPose(station, section);
        this.station = station;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();
        Pose2d extraLeftPoint = new Pose2d(
                Landmarks.BlueFarLeftBranchB.getX(),
                Landmarks.BlueFarLeftBranchB.getY() + 0.9144,
                Landmarks.BlueFarLeftBranchB.getRotation());
        Pose2d extraRightPoint = new Pose2d(
                Landmarks.BlueFarRightBranchA.getX(),
                Landmarks.BlueFarRightBranchA.getY() - 0.9144,
                Landmarks.BlueFarLeftBranchA.getRotation());
        if (station == Landmarks.CoralStation.LEFT) {
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(extraLeftPoint), 10));
        }
        else {
            // scared of going right on accident because testing, so commenting out
//            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(extraRightPoint), 10));
        }
        this.logic.setKeyPoints(swervePoints);
        if (kinematics) {
            this.logic.setGlobalKinematicValues(new SwervePointKinematics(2, 0, 0, 2.5));
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        }
        else {
            this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.ConstantVelocity);
        }
        super.initialize();
    }
}
