package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.oracle.ReefRoutingCircle;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DoubleProperty;
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

public class DriveToBargeCommand extends SwerveSimpleTrajectoryCommand {
    public DoubleProperty distanceFromBarge;
    public DoubleProperty blueRobotMin;
    public DoubleProperty redRobotMin;
    private final ReefRoutingCircle routingCircleBlue;
    private final ReefRoutingCircle routingCircleRed;
    private final ElectricalContract electricalContract;

    @Inject
    public DriveToBargeCommand(BaseSwerveDriveSubsystem drive, PoseSubsystem pose,
                               PropertyFactory pf,
                               HeadingModule.HeadingModuleFactory headingModuleFactory,
                               RobotAssertionManager robotAssertionManager,
                               CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem,
                               ElectricalContract electricalContract) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
        this.electricalContract = electricalContract;
        pf.setPrefix("DriveToBargeCommand");
        Translation2d center = Landmarks.BlueCenterOfReef.getTranslation();

        routingCircleBlue = new ReefRoutingCircle(center, 2);
        routingCircleRed = new ReefRoutingCircle(PoseSubsystem.convertBlueToRed(center), 2);

        distanceFromBarge = pf.createPersistentProperty("distanceFromBarge-m", 1);
        blueRobotMin = pf.createPersistentProperty("blueRobotMin-m", 4.20);
        redRobotMin = pf.createPersistentProperty("redRobotMin-m", 3.85);
    }

    public Pose2d getNearestPoint(
            double startY, DriverStation.Alliance alliance) {


        // Determine if the alliance is BLUE.
        boolean isBlue = alliance == DriverStation.Alliance.Blue;

        // Snap startY to the nearest boundary if it falls between the red and blue
        // minimum values.
        double radiusOfRobotFromCenter = electricalContract.getRadiusOfRobot().div(2).in(Meters);
        if (isBlue) {
            double blueMinimumWithBumper = blueRobotMin.get() + radiusOfRobotFromCenter;
            if (startY <= blueMinimumWithBumper) {
                startY = blueMinimumWithBumper;

            }
        } else if (startY >= (redRobotMin.get() - radiusOfRobotFromCenter)) {
            startY = redRobotMin.get() - radiusOfRobotFromCenter;
        }

        // Compute the X coordinate based on the alliance.
        double x = isBlue ? PoseSubsystem.fieldXMidpointInMeters
                .in(Meters) - distanceFromBarge.get()
                : PoseSubsystem.fieldXMidpointInMeters.in(Meters)
                + distanceFromBarge.get();

        // Set heading to 180 if blue, otherwise 0.
        double heading = isBlue ? 180 : 0;

        // Return the computed Pose2d.
        return new Pose2d(x, startY, Rotation2d.fromDegrees(heading));
    }

    @Override
    public void initialize() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        Pose2d goal = getNearestPoint(pose.getCurrentPose2d().getY(),
                alliance
        );

        List<XbotSwervePoint> swervePoints =
                alliance.equals(DriverStation.Alliance.Blue)
                        ? routingCircleBlue.generateSwervePoints(pose.getCurrentPose2d(), goal) :
                        routingCircleRed.generateSwervePoints(pose.getCurrentPose2d(), goal);
        super.logic.setPrioritizeRotationIfCloseToGoal(true);
        super.logic.setKeyPoints(swervePoints);
        super.logic.setVelocityMode(
                SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        super.logic.setGlobalKinematicValues(
                new SwervePointKinematics(2, 1, 0, 4.5));
        super.initialize();
    }
}
