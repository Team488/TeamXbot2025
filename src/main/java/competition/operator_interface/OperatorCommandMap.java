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
        operatorInterface.driverGamepad.getifAvailable(1).onTrue(resetHeading);

        // Below are for testing purposes only!!!
        SwervePointKinematics kinematicValuesForTesting = new SwervePointKinematics(0.5, 0, 0, 2);

        var command2 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points2 = new ArrayList<>();

        XbotSwervePoint p4 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(2.5, 0), new Rotation2d(0), 10);

        points2.add(p4);
        command2.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command2.logic.setKeyPoints(points2);
        command2.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);

        var command3 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points3 = new ArrayList<>();

        XbotSwervePoint p5 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(0, 0), Rotation2d.fromDegrees(45), 10);

        points3.add(p5);
        command3.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command3.logic.setKeyPoints(points3);
        command3.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);

        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(command2); // 3 One thingy
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.Y).onTrue(command3);
    }

    @Inject
    public void setupSimulatorCommands(
        ResetSimulatedPose resetPose
    ) {
        resetPose.includeOnSmartDashboard();
    }
}
