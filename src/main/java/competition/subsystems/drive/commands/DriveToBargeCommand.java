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
           double startX, double startY, DriverStation.Alliance alliance) {


        // Determine if the alliance is BLUE.
        boolean isBlue = alliance == DriverStation.Alliance.Blue;

        // Snap startY to the nearest boundary if it falls between the red and blue
        // minimum values.
        double radius = electricalContract.getRadiusOfRobot().div(2).in(Meters);
        if (isBlue) {
            startY = Math.max(startY, blueRobotMin.get() + radius);
        } else {
            startY = Math.min(startY, redRobotMin.get() - radius);
        }

        // Set heading to 180 if blue, otherwise 0.
        double heading;
        if(startX < PoseSubsystem.fieldXMidpointInMeters
                .in(Meters)) {
            startX = PoseSubsystem.fieldXMidpointInMeters.in(Meters) - distanceFromBarge.get();
            heading = 180;
        } else {
            startX = PoseSubsystem.fieldXMidpointInMeters.in(Meters) + distanceFromBarge.get();
            heading = 0;
        }


        // Return the computed Pose2d.
        return new Pose2d(startX, startY, Rotation2d.fromDegrees(heading));
    }

    @Override
    public void initialize() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        Pose2d currentPose = pose.getCurrentPose2d();
        Pose2d goal = getNearestPoint(currentPose.getX(), currentPose.getY(),
                alliance
        );

        List<XbotSwervePoint> swervePoints =
                 currentPose.getX() < PoseSubsystem.fieldXMidpointInMeters.in(Meters)
                        ? routingCircleBlue.generateSwervePoints(pose.getCurrentPose2d(), goal) :
                        routingCircleRed.generateSwervePoints(pose.getCurrentPose2d(), goal);
        super.logic.setPrioritizeRotationIfCloseToGoal(true);
        super.logic.setKeyPoints(swervePoints);
        super.logic.setRotationPrioritizationScaleback(0.5);
        super.logic.setVelocityMode(
                SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        super.logic.setGlobalKinematicValues(
                new SwervePointKinematics(3, 2, 0, 4.5));
        super.initialize();
    }
}
