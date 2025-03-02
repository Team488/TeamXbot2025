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
    boolean addPoint = false;

    @Inject
    public DriveToCoralStationSectionCommand(DriveSubsystem drive, PoseSubsystem pose,
                                             PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                             RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
    }

    public void setTargetCoralStationSection(Landmarks.CoralStation station, Landmarks.CoralStationSection section, boolean addPoint) {
        this.targetCoralStationSection = Landmarks.getCoralStationSectionPose(station, section);
        this.addPoint = addPoint;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();
        Pose2d extraPoint = new Pose2d(
                Landmarks.BlueCloseLeftBranchA.getX(),
                Landmarks.BlueCloseLeftBranchA.getY() + 1.207008,
                Landmarks.BlueFarLeftBranchB.getRotation());
        if (addPoint) {
            swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(extraPoint), 10));
        }
        swervePoints.add(new XbotSwervePoint(PoseSubsystem.convertBlueToRedIfNeeded(targetCoralStationSection), 10));
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
