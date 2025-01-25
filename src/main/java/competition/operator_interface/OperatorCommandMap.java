package competition.operator_interface;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.simulation.commands.ResetSimulatedPose;
import competition.subsystems.drive.commands.DebugSwerveModuleCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.subsystems.drive.swerve.commands.ChangeActiveSwerveModuleCommand;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;
import competition.subsystems.drive.commands.DriveToWaypointsWithVisionCommand;

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
            DriveToWaypointsWithVisionCommand driveToWaypointsCommand) {
        resetHeading.setHeadingToApply(0);
        operatorInterface.gamepad.getifAvailable(1).onTrue(resetHeading);
        operatorInterface.gamepad.getifAvailable(XXboxController.XboxButton.RightBumper).onTrue(driveToWaypointsCommand);
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
        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(changeActiveModule);
        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.A).onTrue(debugModule);
        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(typicalSwerveDrive);
    }

    @Inject
    public void setupSimulatorCommands(
        ResetSimulatedPose resetPose
    ) {
        resetPose.includeOnSmartDashboard();
    }
}
