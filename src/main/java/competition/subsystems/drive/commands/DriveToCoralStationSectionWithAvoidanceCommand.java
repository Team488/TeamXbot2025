package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefRoutingCircle;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class DriveToCoralStationSectionWithAvoidanceCommand extends SwerveSimpleTrajectoryCommand {

    boolean kinematics = true;
    Pose2d currentPose;
    Pose2d targetCoralStationSectionPose;
    ReefRoutingCircle routingCircle;

    @Inject
    public DriveToCoralStationSectionWithAvoidanceCommand(DriveSubsystem drive, PoseSubsystem pose,
                                             PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                             RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
        this.pose = pose;
        Translation2d center = Landmarks.BlueCenterOfReef.getTranslation();
        double radius = 2.0;
        routingCircle = new ReefRoutingCircle(center, radius);
    }

    public void setTargetCoralStationSection(Landmarks.CoralStation station, Landmarks.CoralStationSection section) {
        this.targetCoralStationSectionPose = Landmarks.getCoralStationSectionPose(station, section);
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        currentPose = pose.getCurrentPose2d();
        List<XbotSwervePoint> swervePoints = routingCircle.generateSwervePoints(currentPose, targetCoralStationSectionPose);
        this.logic.setKeyPoints(swervePoints);
        if (kinematics) {
            this.logic.setGlobalKinematicValues(new SwervePointKinematics(0.5, 0, 0, 2));
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        }
        else {
            this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.ConstantVelocity);
        }
        super.initialize();
    }
}
