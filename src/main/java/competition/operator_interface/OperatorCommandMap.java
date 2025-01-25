package competition.operator_interface;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;

import competition.simulation.commands.ResetSimulatedPose;
import competition.subsystems.drive.commands.DebugSwerveModuleCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.pose.commands.ToTestLocationCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.swerve.commands.ChangeActiveSwerveModuleCommand;
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
    
    // Example for setting up a command to fire when a button is pressed:
    @Inject
    public void setupMyCommands(
            OperatorInterface operatorInterface,
            SetRobotHeadingCommand resetHeading,
            Provider<SwerveSimpleTrajectoryCommand> provider,
            ToTestLocationCommand locationCommand) {
        resetHeading.setHeadingToApply(0);
        //operatorInterface.gamepad.getifAvailable(1).onTrue(resetHeading);

        var toOrigin = provider.get();
        List<XbotSwervePoint> toOriginPoints = new ArrayList<>();
        toOriginPoints.add(new XbotSwervePoint(new Translation2d(6,2), new Rotation2d(0), 10));
        toOriginPoints.add(new XbotSwervePoint(new Translation2d(2,2), new Rotation2d(0), 10));
        toOrigin.logic.setEnableConstantVelocity(true);
        toOrigin.logic.setConstantVelocity(2);
        toOrigin.logic.setKeyPoints(toOriginPoints);

        var toGoal = provider.get();
        List<XbotSwervePoint> toGoalPoints = new ArrayList<>();
        toGoalPoints.add(new XbotSwervePoint(new Translation2d(14,2), new Rotation2d(0), 10));
        toGoalPoints.add(new XbotSwervePoint(new Translation2d(2,2), new Rotation2d(0), 10));
        toGoal.logic.setEnableConstantVelocity(true);
        toGoal.logic.setConstantVelocity(2);
        toGoal.logic.setKeyPoints(toGoalPoints);

        operatorInterface.gamepad.getifAvailable(XXboxController.XboxButton.A).onTrue(locationCommand);
        operatorInterface.gamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(toOrigin);
        operatorInterface.gamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(toGoal);
    }

    // Programmer commands are only meant to be used to debug or test the robot. They should not be used in competition,
    // and many do dangerous things like bypass various safeties or force the robot into states that aren't useful
    // (e.g. only driving a single swerve module at a time for testing purposes).
    @Inject
    public void setupProgrammerCommands(
            OperatorInterface oi,
            DebugSwerveModuleCommand debugModule,
            ChangeActiveSwerveModuleCommand changeActiveModule,
            SwerveDriveWithJoysticksCommand typicalSwerveDrive) {
//        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(changeActiveModule);
//        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.A).onTrue(debugModule);
//        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(typicalSwerveDrive);
    }

    @Inject
    public void setupSimulatorCommands(
        ResetSimulatedPose resetPose
    ) {
        resetPose.includeOnSmartDashboard();
    }
}
