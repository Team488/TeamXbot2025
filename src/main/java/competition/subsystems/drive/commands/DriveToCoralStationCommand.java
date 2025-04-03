package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.oracle.ReefRoutingCircle;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.List;

import static edu.wpi.first.units.Units.Meters;

public class DriveToCoralStationCommand extends SwerveSimpleTrajectoryCommand {

    public Landmarks.CoralStation coralStation = Landmarks.CoralStation.LEFT;
    private final ReefRoutingCircle routingCircleBlue;
    private final ReefRoutingCircle routingCircleRed;

    @Inject
    public DriveToCoralStationCommand(BaseSwerveDriveSubsystem drive, PoseSubsystem pose,
                                      PropertyFactory pf,
                                      HeadingModule.HeadingModuleFactory headingModuleFactory,
                                      RobotAssertionManager robotAssertionManager,
                                      CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem,
                                      ElectricalContract electricalContract) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
        pf.setPrefix("DriveToCoralStationCommand");
        Translation2d center = Landmarks.BlueCenterOfReef.getTranslation();
        routingCircleBlue = new ReefRoutingCircle(center, 2);
        routingCircleRed = new ReefRoutingCircle(PoseSubsystem.convertBlueToRed(center), 2);
    }

    public DriveToCoralStationCommand setCoralStation(Landmarks.CoralStation coralStation) {
        this.coralStation = coralStation;
        return this;
    }

    @Override
    public void initialize() {

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        Pose2d goal = coralStation.equals(Landmarks.CoralStation.LEFT) ? PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueLeftCoralStationMid) :
                PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueRightCoralStationMid);

        List<XbotSwervePoint> swervePoints =
                pose.getCurrentPose2d().getX() < PoseSubsystem.fieldXMidpointInMeters.in(Meters)
                        ? routingCircleBlue.generateSwervePoints(pose.getCurrentPose2d(), goal) :
                        routingCircleRed.generateSwervePoints(pose.getCurrentPose2d(), goal);
            super.logic.setKeyPoints(swervePoints);
        super.logic.setVelocityMode(
                SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        // End at coral station with some velocity
        super.logic.setGlobalKinematicValues(
                new SwervePointKinematics(3, 3, 0.25, 4.5));
        super.initialize();
    }
}
