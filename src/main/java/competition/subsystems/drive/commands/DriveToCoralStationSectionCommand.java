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

public class DriveToCoralStationSectionCommand extends SwerveSimpleTrajectoryCommand {

    Pose2d targetCoralStationSection;

    @Inject
    public DriveToCoralStationSectionCommand(DriveSubsystem drive, PoseSubsystem pose,
                                             PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                             RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
    }

    public void setTargetCoralStationSection(Landmarks.CoralStation station, Landmarks.CoralStationSection section) {
        this.targetCoralStationSection = Landmarks.getCoralStationSectionPose(station, section);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();
        swervePoints.add(new XbotSwervePoint(targetCoralStationSection, 10));
        this.logic.setKeyPoints(swervePoints);
        this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.ConstantVelocity);
//        this.logic.setGlobalKinematicValues(new SwervePointKinematics(.5, 0, 0, 2));
//        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}
