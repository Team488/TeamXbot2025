package competition.operator_interface;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;

import competition.simulation.commands.ResetSimulatedPose;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;
import xbot.common.trajectory.XbotSwervePoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Maps operator interface buttons to commands
 */
@Singleton
public class OperatorCommandMap {

    @Inject
    public OperatorCommandMap() {}

    @Inject
    public void setupMyCommands(
            OperatorInterface operatorInterface,
            SetRobotHeadingCommand resetHeading,
            Provider<SwerveSimpleTrajectoryCommand> swerveSimpleTrajectoryCommandProvider) {
        resetHeading.setHeadingToApply(0);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.A).onTrue(resetHeading);

        // Below are for testing purposes only!!!
        SwervePointKinematics kinematicValuesForTesting = new SwervePointKinematics(2, 0, 0, 4.5);

        // Command 1: 5 meter route
        var command1 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points1 = new ArrayList<>();
        XbotSwervePoint command1Point1 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(9, 0), new Rotation2d(0), 10);
        points1.add(command1Point1);
        command1.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command1.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        command1.logic.setKeyPoints(points1);

        // Command 2: to origin
        var command2 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points2 = new ArrayList<>();
        XbotSwervePoint command2Point1 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(0, 0), new Rotation2d(0), 10);
        points2.add(command2Point1);
        command2.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command2.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        command2.logic.setKeyPoints(points2);

        // Command 3: 8 meter route
        var command3 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points3 = new ArrayList<>();
        XbotSwervePoint command3Point1 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(8, 0), new Rotation2d(0), 10);
        points3.add(command3Point1);
        command3.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command3.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        command3.logic.setKeyPoints(points3);

        // Command 4: the multi-point
        var command4 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points4 = new ArrayList<>();
        XbotSwervePoint command4Point1 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(1.5, 0), new Rotation2d(0), 10);
        XbotSwervePoint command4Point2 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(3, -1.5), new Rotation2d(45), 10);
        points4.add(command4Point1);
        points4.add(command4Point2);
        command4.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command4.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        command4.logic.setKeyPoints(points4);

        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(command1); // THE 5 METER
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.Y).onTrue(command2); // TO ORIGIN
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(command4); // MULTI-POINT
    }

    @Inject
    public void setupSimulatorCommands(
        ResetSimulatedPose resetPose
    ) {
        resetPose.includeOnSmartDashboard();
    }

}
