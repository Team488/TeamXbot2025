package competition.operator_interface;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.simulation.commands.ResetSimulatedPose;
import competition.subsystems.algae_collection.commands.AlgaeCollectionIntakeCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionOutputCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionStopCommand;
import competition.subsystems.coral_scorer.commands.IntakeCoralCommand;
import competition.subsystems.coral_scorer.commands.ScoreCoralCommand;
import competition.subsystems.coral_scorer.commands.StopCoralCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.DebugSwerveModuleCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.subsystems.drive.swerve.commands.ChangeActiveSwerveModuleCommand;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;

import java.util.Set;

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
            SetRobotHeadingCommand resetHeading) {
        resetHeading.setHeadingToApply(0);
        operatorInterface.gamepad.getifAvailable(1).onTrue(resetHeading);
    }

    // Programmer commands are only meant to be used to debug or test the robot. They should not be used in competition,
    // and many do dangerous things like bypass various safeties or force the robot into states that aren't useful
    // (e.g. only driving a single swerve module at a time for testing purposes).
    @Inject
    public void setupProgrammerCommands(
            OperatorInterface oi,
            DebugSwerveModuleCommand debugModule,
            ChangeActiveSwerveModuleCommand changeActiveModule,
            SwerveDriveWithJoysticksCommand typicalSwerveDrive,
            IntakeCoralCommand intakeCoralCommand,
            ScoreCoralCommand scoreCoralCommand,
            StopCoralCommand stopCoralCommand,
            AlgaeCollectionIntakeCommand algaeCollectionIntakeCommand,
            AlgaeCollectionOutputCommand algaeCollectionOutputCommand,
            AlgaeCollectionStopCommand algaeCollectionStopCommand,
            DriveSubsystem drive) {
        oi.programmerGamepad.getPovIfAvailable(0).onTrue(changeActiveModule);
        oi.programmerGamepad.getPovIfAvailable(90).onTrue(debugModule);
        oi.programmerGamepad.getPovIfAvailable(180).onTrue(typicalSwerveDrive);

        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper)
                .whileTrue(new DeferredCommand(() -> drive.getActiveSwerveModuleSubsystem()
                        .getSteeringSubsystem()
                        .sysIdQuasistatic(SysIdRoutine.Direction.kForward), Set.of()));
        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.RightBumper)
                .whileTrue(new DeferredCommand(() -> drive.getActiveSwerveModuleSubsystem()
                        .getSteeringSubsystem()
                        .sysIdQuasistatic(SysIdRoutine.Direction.kReverse), Set.of()));

        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.LeftTrigger).whileTrue(intakeCoralCommand);
        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.RightTrigger).whileTrue(scoreCoralCommand);

        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(algaeCollectionIntakeCommand);
        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.B).whileTrue(algaeCollectionOutputCommand);

    }

    @Inject
    public void setupSimulatorCommands(
        ResetSimulatedPose resetPose
    ) {
        resetPose.includeOnSmartDashboard();
    }
}
