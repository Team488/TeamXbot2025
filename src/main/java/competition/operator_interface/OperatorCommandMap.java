package competition.operator_interface;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.simulation.commands.ResetSimulatedPose;
import competition.subsystems.algae_collection.commands.AlgaeCollectionIntakeCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionOutputCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionStopCommand;
import competition.subsystems.coral_arm.commands.ForceCoralPivotCalibrated;
import competition.subsystems.coral_arm.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.coral_scorer.commands.IntakeCoralCommand;
import competition.subsystems.coral_scorer.commands.ScoreCoralCommand;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.coral_scorer.commands.StopCoralCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DebugSwerveModuleCommand;
import competition.subsystems.drive.commands.DriveToWaypointsWithVisionCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.TeleportToPositionCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.ForceElevatorCalibratedCommand;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import competition.subsystems.oracle.commands.DriveAccordingToOracleCommand;
import competition.subsystems.oracle.commands.SuperstructureAccordingToOracleCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.subsystems.drive.SwerveBezierTrajectoryCommand;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.swerve.commands.ChangeActiveSwerveModuleCommand;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Seconds;

/**
 * Maps operator interface buttons to commands
 */
@Singleton
public class OperatorCommandMap {

    @Inject
    public OperatorCommandMap() {}

    @Inject
    public void setupDriverCommands(
            OperatorInterface operatorInterface,
            SetRobotHeadingCommand resetHeading,
            SwerveBezierTrajectoryCommand sbtc) {
        resetHeading.setHeadingToApply(0);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.A).onTrue(resetHeading);

        List<Translation2d> controlPoints = new ArrayList<>();
        controlPoints.add(new Translation2d(7, 3));
        controlPoints.add(new Translation2d(7, 1));
        controlPoints.add(new Translation2d(3, 1));

        sbtc.setBezierConfiguration(controlPoints, new Pose2d(2, 1, Rotation2d.fromDegrees(0)), 5);
        sbtc.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        sbtc.logic.setGlobalKinematicValues(new SwervePointKinematics(0.5, 0, 0, 2));

        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(sbtc);
    }



    @Inject
    public void setUpOperatorCommands(OperatorInterface oi,
                                      PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                      ScoreCoralCommand scoreCoralCommand, IntakeCoralCommand intakeCoralCommand,
                                      SetCoralArmTargetAngleCommand setCoralArmTargetAngleCommand,
                                      ScoreWhenReadyCommand scoreWhenReadyCommand) {
        var prepL4 = prepCoralSystemCommandGroupFactory.create(Landmarks.CoralLevel.FOUR);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.Y).onTrue(prepL4);

        var prepL2 = prepCoralSystemCommandGroupFactory.create(Landmarks.CoralLevel.TWO);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).onTrue(prepL2);

        var homed = prepCoralSystemCommandGroupFactory.create(Landmarks.CoralLevel.COLLECTING);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(homed);

        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(scoreWhenReadyCommand);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.A).whileTrue(intakeCoralCommand);

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
            ForceElevatorCalibratedCommand forceElevatorCalibratedCommand,
            ForceCoralPivotCalibrated forceCoralPivotCalibratedCommand) {

        var returnToBase = setElevatorTargetHeightCommandProvider.get();
        returnToBase.setHeight(Landmarks.CoralLevel.COLLECTING);
        var riseToL2 = setElevatorTargetHeightCommandProvider.get();
        riseToL2.setHeight(Landmarks.CoralLevel.TWO);
        var riseToL3 = setElevatorTargetHeightCommandProvider.get();
        riseToL3.setHeight(Landmarks.CoralLevel.THREE);
        var riseToL4 = setElevatorTargetHeightCommandProvider.get();
        riseToL4.setHeight(Landmarks.CoralLevel.FOUR);

        var riseToScore = setArmTargetAngleCommandProvider.get();
        riseToScore.setAngle(Landmarks.CoralLevel.FOUR);
        var lowerToHumanLoad = setArmTargetAngleCommandProvider.get();
        lowerToHumanLoad.setAngle(Landmarks.CoralLevel.COLLECTING);

        /*
        oi.superstructureGamepad.getPovIfAvailable(0).onTrue(changeActiveModule);
        oi.superstructureGamepad.getPovIfAvailable(90).onTrue(debugModule);
        oi.superstructureGamepad.getPovIfAvailable(180).onTrue(typicalSwerveDrive);
        */
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.LeftTrigger).whileTrue(intakeCoralCommand);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.RightTrigger).whileTrue(scoreCoralCommand);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper).onTrue(lowerToHumanLoad);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).onTrue(riseToScore);
      
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Start).onTrue(forceElevatorCalibratedCommand);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.A).whileTrue(riseToL2);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.B).whileTrue(riseToL3);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(riseToL4);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Back).onTrue(forceCoralPivotCalibratedCommand);

//        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(algaeCollectionIntakeCommand);
//        oi.programmerGamepad.getifAvailable(XXboxController.XboxButton.B).whileTrue(algaeCollectionOutputCommand);

    }

    @Inject
    public void setupSysIdCommands(
        OperatorInterface oi,
        DriveSubsystem drive,
        ElevatorSubsystem elevator
    ) {

        oi.algaeAndSysIdGamepad.getifAvailable(XXboxController.XboxButton.A)
                .whileTrue(drive.sysIdQuasistaticRotation(SysIdRoutine.Direction.kForward)
                        .andThen(new WaitCommand(Seconds.of(1)))
                        .andThen(drive.sysIdQuasistaticRotation(SysIdRoutine.Direction.kReverse))
                        .andThen(new WaitCommand(Seconds.of(1)))
                        .andThen(drive.sysIdDynamicRotation(SysIdRoutine.Direction.kForward))
                        .andThen(new WaitCommand(Seconds.of(1)))
                        .andThen(drive.sysIdDynamicRotation(SysIdRoutine.Direction.kReverse)));
        oi.algaeAndSysIdGamepad.getifAvailable(XXboxController.XboxButton.B)
                .whileTrue(drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward)
                        .andThen(new WaitCommand(Seconds.of(1)))
                        .andThen(drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse))
                        .andThen(new WaitCommand(Seconds.of(1)))
                        .andThen(drive.sysIdDynamicDrive(SysIdRoutine.Direction.kForward))
                        .andThen(new WaitCommand(Seconds.of(1)))
                        .andThen(drive.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse)));

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
