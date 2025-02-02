package competition.operator_interface;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;

import competition.simulation.commands.ResetSimulatedPose;
import competition.subsystems.algae_collection.commands.AlgaeCollectionIntakeCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionOutputCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionStopCommand;
import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_arm_pivot.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.coral_scorer.commands.IntakeCoralCommand;
import competition.subsystems.coral_scorer.commands.ScoreCoralCommand;
import competition.subsystems.coral_scorer.commands.StopCoralCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.DebugSwerveModuleCommand;
import competition.subsystems.drive.commands.DriveToWaypointsWithVisionCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;

import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.ForceElevatorCalibratedCommand;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import xbot.common.controls.sensors.XXboxController;
import xbot.common.subsystems.drive.swerve.commands.ChangeActiveSwerveModuleCommand;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;

import static edu.wpi.first.units.Units.Seconds;

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
            SetRobotHeadingCommand resetHeading, DriveToWaypointsWithVisionCommand driveWaypoints) {
        resetHeading.setHeadingToApply(0);
        operatorInterface.driverGamepad.getifAvailable(1).onTrue(resetHeading);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).onTrue(driveWaypoints);
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
            Provider<SetCoralArmTargetAngleCommand> setArmTargetAngleCommandProvider,
            Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider,
            ForceElevatorCalibratedCommand forceElevatorCalibratedCommand ) {

        var riseToL2 = setElevatorTargetHeightCommandProvider.get();
        riseToL2.setHeight(ElevatorSubsystem.ElevatorGoals.ScoreL2);
        var riseToL3 = setElevatorTargetHeightCommandProvider.get();
        riseToL3.setHeight(ElevatorSubsystem.ElevatorGoals.ScoreL3);
        var riseToL4 = setElevatorTargetHeightCommandProvider.get();
        riseToL4.setHeight(ElevatorSubsystem.ElevatorGoals.ScoreL4);

        var riseToScore = setArmTargetAngleCommandProvider.get();
        riseToScore.setAngle(CoralArmPivotSubsystem.ArmGoals.Score);
        var lowerToHumanLoad = setArmTargetAngleCommandProvider.get();
        lowerToHumanLoad.setAngle(CoralArmPivotSubsystem.ArmGoals.HumanLoad);
      
        oi.superstructureGamepad.getPovIfAvailable(0).onTrue(changeActiveModule);
        oi.superstructureGamepad.getPovIfAvailable(90).onTrue(debugModule);
        oi.superstructureGamepad.getPovIfAvailable(180).onTrue(typicalSwerveDrive);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.LeftTrigger).whileTrue(intakeCoralCommand);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.RightTrigger).whileTrue(scoreCoralCommand);

        //oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(lowerToHumanLoad);
        //oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Y).whileTrue(riseToScore);
      
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Start).onTrue(forceElevatorCalibratedCommand);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.B).whileTrue(riseToL2);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.A).whileTrue(riseToL3);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(riseToL4);

//        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(algaeCollectionIntakeCommand);
//        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.B).whileTrue(algaeCollectionOutputCommand);

    }

    @Inject
    public void setupSysIdCommands(
        OperatorInterface oi,
        DriveSubsystem drive,
        ElevatorSubsystem elevator
    ) {
//        oi.algaeAndSysIdGamepad.getifAvailable(XXboxController.XboxButton.A)
//                .whileTrue(drive.sysIdQuasistaticRotation(SysIdRoutine.Direction.kForward)
//                        .andThen(new WaitCommand(Seconds.of(1)))
//                        .andThen(drive.sysIdQuasistaticRotation(SysIdRoutine.Direction.kReverse))
//                        .andThen(new WaitCommand(Seconds.of(1)))
//                        .andThen(drive.sysIdDynamicRotation(SysIdRoutine.Direction.kForward))
//                        .andThen(new WaitCommand(Seconds.of(1)))
//                        .andThen(drive.sysIdDynamicRotation(SysIdRoutine.Direction.kReverse)));
//        oi.algaeAndSysIdGamepad.getifAvailable(XXboxController.XboxButton.B)
//                .whileTrue(drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward)
//                        .andThen(new WaitCommand(Seconds.of(1)))
//                        .andThen(drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse))
//                        .andThen(new WaitCommand(Seconds.of(1)))
//                        .andThen(drive.sysIdDynamicDrive(SysIdRoutine.Direction.kForward))
//                        .andThen(new WaitCommand(Seconds.of(1)))
//                        .andThen(drive.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse)));

        oi.algaeAndSysIdGamepad.getPovIfAvailable(0).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        oi.algaeAndSysIdGamepad.getPovIfAvailable(90).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        oi.algaeAndSysIdGamepad.getPovIfAvailable(180).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        oi.algaeAndSysIdGamepad.getPovIfAvailable(270).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // Not used, but leaving these here as a sample of how to use a DeferredCommand
//        oi.sysIdGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper)
//                .whileTrue(new DeferredCommand(() -> drive.getActiveSwerveModuleSubsystem()
//                        .getSteeringSubsystem()
//                        .sysIdQuasistatic(SysIdRoutine.Direction.kForward), Set.of()));
//        oi.sysIdGamepad.getifAvailable(XXboxController.XboxButton.RightBumper)
//                .whileTrue(new DeferredCommand(() -> drive.getActiveSwerveModuleSubsystem()
//                        .getSteeringSubsystem()
//                        .sysIdQuasistatic(SysIdRoutine.Direction.kReverse), Set.of()));

    }

    @Inject
    public void setupSimulatorCommands(
        ResetSimulatedPose resetPose
    ) {
        resetPose.includeOnSmartDashboard();
    }
}
