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
        operatorInterface.gamepad.getifAvailable(1).onTrue(resetHeading);

        // Below are for testing purposes only!!!
        SwervePointKinematics kinematicValuesForTesting = new SwervePointKinematics(0.5, 0, 0, 2);

        // Command 1: to (1,0) then back to (0,0)
        var command1 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points1 = new ArrayList<>();

        XbotSwervePoint p1 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(1, 0), new Rotation2d(), 10);
        p1.setKinematics(kinematicValuesForTesting);
        XbotSwervePoint p2 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(0, 0), new Rotation2d(), 10);
        p2.setKinematics(kinematicValuesForTesting);

        points1.add(p1);
        points1.add(p2);
        command1.logic.setKeyPoints(points1);
        command1.logic.setVelocityMode(SwerveSimpleTrajectoryMode.KinematicsForIndividualPoints);

        var command2 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points2 = new ArrayList<>();

        XbotSwervePoint p3 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(6, 2), new Rotation2d(), 10);
        XbotSwervePoint p4 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(2, 2), new Rotation2d(), 10);

        points2.add(p3);
        points2.add(p4);
        command2.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command2.logic.setKeyPoints(points2);
        command2.logic.setVelocityMode(SwerveSimpleTrajectoryMode.KinematicsForPointsList);

        var command3 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points3 = new ArrayList<>();

        XbotSwervePoint p5 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(14, 2), new Rotation2d(), 10);
        XbotSwervePoint p6 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(2, 2), new Rotation2d(), 10);

        points3.add(p5);
        points3.add(p6);
        command3.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command3.logic.setKeyPoints(points3);
        command3.logic.setVelocityMode(SwerveSimpleTrajectoryMode.KinematicsForPointsList);

        // Command 4: Far drive for observation purposes
        var command4 = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points4 = new ArrayList<>();

        XbotSwervePoint p7 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(2.5, 0), new Rotation2d(), 10);
        XbotSwervePoint p8 = XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(new Translation2d(0, 0), new Rotation2d(), 10);

        points4.add(p7);
        points4.add(p8);
        command4.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        command4.logic.setKeyPoints(points4);
        command4.logic.setVelocityMode(SwerveSimpleTrajectoryMode.KinematicsForPointsList);

        //operatorInterface.gamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(command4); // 2 Invididual kinematics values
        operatorInterface.gamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(command2); // 3 One thingy
        operatorInterface.gamepad.getifAvailable(XXboxController.XboxButton.Y).onTrue(command3);
    }

    @Inject
    public void setupSimulatorCommands(
        ResetSimulatedPose resetPose
    ) {
        resetPose.includeOnSmartDashboard();
    }
}
