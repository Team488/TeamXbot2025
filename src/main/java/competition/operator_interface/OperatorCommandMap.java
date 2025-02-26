package competition.operator_interface;

import competition.commandgroups.HeadingAssistedDriveAndScoreCommandGroup;
import competition.commandgroups.PrepAlgaeSystemCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.simulation.commands.ResetSimulatedPose;
import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import competition.subsystems.algae_arm.commands.ForceAlgaeArmCalibrated;
import competition.subsystems.algae_arm.commands.SetAlgaeArmSetpointToTargetPosition;
import competition.subsystems.algae_collection.commands.AlgaeCollectionIntakeCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionOutputCommand;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_arm.commands.ForceCoralArmCalibratedCommand;
import competition.subsystems.coral_arm.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.coral_scorer.commands.IntakeCoralCommand;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.coral_scorer.commands.ScoreCoralCommand;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.CalibrateDriveCommand;
import competition.subsystems.drive.commands.DebugSwerveModuleCommand;
import competition.subsystems.drive.commands.DriveToCoralStationWithVisionCommand;
import competition.subsystems.drive.commands.DriveToLocationWithPID;
import competition.subsystems.drive.commands.DriveWithSnapToTagCommand;
import competition.subsystems.drive.commands.RotateToHeadingWithHeadingModule;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.TeleportToPositionCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.ForceElevatorCalibratedCommand;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import competition.subsystems.oracle.commands.DriveAccordingToOracleCommand;
import competition.subsystems.oracle.commands.SuperstructureAccordingToOracleCommand;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.commands.ResetPoseCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.swerve.commands.ChangeActiveSwerveModuleCommand;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;
import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Degree;

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
            Provider<AlignToReefWithAprilTagCommand> alignToReefWithAprilTagProvider,
            Provider<SwerveSimpleTrajectoryCommand> swerveSimpleTrajectoryCommandProvider,
            Provider<DriveToLocationWithPID> driveToLocationWithPIDProvider,
            Provider<RotateToHeadingWithHeadingModule> rotationToHeadingWithHeadingModuleProvider,
            ResetPoseCommand resetPoseCommand,
            DriveAccordingToOracleCommand driveAccordingToOracle,
            SuperstructureAccordingToOracleCommand superstructureAccordingToOracle,
            DriveToCoralStationWithVisionCommand driveToCoralStationWithVisionCommand,
            IntakeCoralCommand intakeCoralCommand,
            SetCoralArmTargetAngleCommand setCoralArmTargetAngleCommand,
            ScoreCoralCommand scoreCoralCommand,
            TeleportToPositionCommand teleportToPositionCommand,
            PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
            ForceElevatorCalibratedCommand forceElevatorCalibratedCommand,
            ForceCoralArmCalibratedCommand forceCoralPivotCalibratedCommand,
            DebugSwerveModuleCommand debugModule,
            ChangeActiveSwerveModuleCommand changeActiveModule,
            SwerveDriveWithJoysticksCommand typicalSwerveDrive,
            HeadingAssistedDriveAndScoreCommandGroup.Factory headingAssistedDriveAndScoreCommandGroupFactory,
            DriveWithSnapToTagCommand driveWithSnapToTagCommand
            ) {
        resetHeading.setHeadingToApply(0);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.Start).onTrue(resetHeading);

        var alignToReefWithAprilTagWithLeftCamera = alignToReefWithAprilTagProvider.get();
        alignToReefWithAprilTagWithLeftCamera.setConfigurations(Cameras.FRONT_LEFT_CAMERA.getIndex(), false, -2);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).whileTrue(alignToReefWithAprilTagWithLeftCamera);

        var alignToReefWithAprilTagWithRightCamera = alignToReefWithAprilTagProvider.get();
        alignToReefWithAprilTagWithRightCamera.setConfigurations(Cameras.FRONT_RIGHT_CAMERA.getIndex(), false, -2);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper).whileTrue(alignToReefWithAprilTagWithRightCamera);

        var oracleControlsRobot = Commands.parallel(driveAccordingToOracle, superstructureAccordingToOracle);

        var homed = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.Y).onTrue(driveToCoralStationWithVisionCommand);
        var branchAHeadingAssistedDriveAndScore = headingAssistedDriveAndScoreCommandGroupFactory.create(Landmarks.Branch.A);
        //operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.A).onTrue(branchAHeadingAssistedDriveAndScore)
        //                .onFalse(homed);
        var branchBHeadingAssistedDriveAndScore = headingAssistedDriveAndScoreCommandGroupFactory.create(Landmarks.Branch.B);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(branchBHeadingAssistedDriveAndScore)
                .onFalse(homed);

        operatorInterface.driverGamepad.getPovIfAvailable(0).onTrue(debugModule);
        operatorInterface.driverGamepad.getPovIfAvailable(90).onTrue(changeActiveModule);
        operatorInterface.driverGamepad.getPovIfAvailable(180).onTrue(typicalSwerveDrive);

        // operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).whileTrue(intakeCoralCommand);
        // operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper).whileTrue(scoreCoralCommand);



        // (BLUE ALLIANCE) Below are different routes to test the SwerveSimpleTrajectoryCommand
        // I don't think createPotentiallyFilppedXbotSwervePoint works under OperatorCommandMap
        SwervePointKinematics kinematicValuesForTesting = new SwervePointKinematics(1, 0, 0, 5);

        var aroundBlueReef = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points1 = new ArrayList<>();
        points1.add(new XbotSwervePoint(new Translation2d(6.5, 6.5), new Rotation2d(0), 10));
        points1.add(new XbotSwervePoint(new Translation2d(2.2, 6.5), new Rotation2d(0), 10));
        points1.add(new XbotSwervePoint(new Translation2d(2.2, 1.5), new Rotation2d(0), 10));
        points1.add(new XbotSwervePoint(new Translation2d(6.5, 1.5), new Rotation2d(0), 10));
        aroundBlueReef.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        aroundBlueReef.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        aroundBlueReef.logic.setKeyPoints(points1);

        var backAndFourth = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points2 = new ArrayList<>();
        points2.add(new XbotSwervePoint(new Translation2d(6.5, 6.5), new Rotation2d(0), 10));
        points2.add(new XbotSwervePoint(new Translation2d(2.2, 6.5), new Rotation2d(0), 10));
        points2.add(new XbotSwervePoint(new Translation2d(6.5, 6.5), new Rotation2d(0), 10));
        points2.add(new XbotSwervePoint(new Translation2d(2.2, 6.5), new Rotation2d(0), 10));
        points2.add(new XbotSwervePoint(new Translation2d(6.5, 6.5), new Rotation2d(0), 10));
        backAndFourth.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        backAndFourth.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        backAndFourth.logic.setKeyPoints(points2);

        var oneLength = swerveSimpleTrajectoryCommandProvider.get();
        List<XbotSwervePoint> points3 = new ArrayList<>();
        points3.add(new XbotSwervePoint(new Translation2d(3, 0), new Rotation2d(0), 10));
        oneLength.logic.setGlobalKinematicValues(kinematicValuesForTesting);
        oneLength.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        oneLength.logic.setKeyPoints(points3);

        oneLength.includeOnSmartDashboard("OneLengthTest");

        aroundBlueReef.includeOnSmartDashboard("AroundReefTest");
        backAndFourth.includeOnSmartDashboard("BackAndForthTest");

        // Don't think this is needed anymore, I'll keep it just in case
        resetPoseCommand.includeOnSmartDashboard("ResetPoseToOriginCommand");

        var driveWithPidNear = driveToLocationWithPIDProvider.get();
        driveWithPidNear.setLocationTarget(new Translation2d(1, 0));

        var driveWithPidFar = driveToLocationWithPIDProvider.get();
        driveWithPidFar.setLocationTarget(new Translation2d(3, 0));

        var rotateTo5Degrees = rotationToHeadingWithHeadingModuleProvider.get();
        rotateTo5Degrees.setTargetHeading(Degree.of(5));

        var rotateTo10Degrees = rotationToHeadingWithHeadingModuleProvider.get();
        rotateTo10Degrees.setTargetHeading(Degree.of(10));

        var rotateTo45Degrees = rotationToHeadingWithHeadingModuleProvider.get();
        rotateTo45Degrees.setTargetHeading(Degree.of(45));

        var rotateTo90Degrees = rotationToHeadingWithHeadingModuleProvider.get();
        rotateTo90Degrees.setTargetHeading(Degree.of(90));

        var rotateTo180Degrees = rotationToHeadingWithHeadingModuleProvider.get();
        rotateTo180Degrees.setTargetHeading(Degree.of(180));

        driveWithPidNear.includeOnSmartDashboard("DriveToLocationWithPIDNear");
        driveWithPidFar.includeOnSmartDashboard("DriveToLocationWithPIDFar");
        rotateTo5Degrees.includeOnSmartDashboard("RotateTo5Degrees");
        rotateTo10Degrees.includeOnSmartDashboard("RotateTo10Degrees");
        rotateTo45Degrees.includeOnSmartDashboard("RotateTo45Degrees");
        rotateTo90Degrees.includeOnSmartDashboard("RotateTo90Degrees");
        rotateTo180Degrees.includeOnSmartDashboard("RotateTo180Degrees");

        driveWithSnapToTagCommand.setChosenTagID(17);
        driveWithSnapToTagCommand.setCameraId(0);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.A).whileTrue(driveWithSnapToTagCommand);
    }



    @Inject
    public void setUpOperatorCommands(OperatorInterface oi,
                                      PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                      ScoreCoralCommand scoreCoralCommand, IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand,
                                      ScoreWhenReadyCommand scoreWhenReadyCommand, ForceElevatorCalibratedCommand forceElevatorCalibratedCommand,
                                      ForceCoralArmCalibratedCommand forceCoralPivotCalibratedCommand,
                                      ForceAlgaeArmCalibrated forceAlgaeArmCalibrated,
                                      Provider<SetAlgaeArmSetpointToTargetPosition> setAlgaeArmProvider,
                                      AlgaeCollectionIntakeCommand intakeAlgae,
                                      AlgaeCollectionOutputCommand ejectAlgae,
                                      CoralArmSubsystem coralArmSubsystem,
                                      PrepAlgaeSystemCommandGroupFactory prepAlgaeSystemCommandGroupFactory) {
        // Coral system buttons
        var prepL4 = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.FOUR);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.Y).onTrue(prepL4);

        var prepL3 = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.THREE);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(prepL3);

        var prepL2 = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.TWO);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.A).onTrue(prepL2);

        var homed = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(homed);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.LeftTrigger).whileTrue(intakeUntilCoralCollectedCommand);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.RightTrigger).whileTrue(scoreCoralCommand);

        // combine all three claibration commands into one parallal command group
        var calibrateAll = Commands.parallel(
                forceElevatorCalibratedCommand,
                forceCoralPivotCalibratedCommand,
                forceAlgaeArmCalibrated).ignoringDisable(true);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.Start).onTrue(calibrateAll);

        // Algae system buttons
        var removeLowAlgae = prepAlgaeSystemCommandGroupFactory.create(AlgaeArmSubsystem.AlgaeArmPositions.ReefAlgaeLow);
        oi.operatorGamepad.getPovIfAvailable(180).onTrue(removeLowAlgae);

        var removeHighAlgae = prepAlgaeSystemCommandGroupFactory.create(AlgaeArmSubsystem.AlgaeArmPositions.ReefAlgaeHigh);
        oi.operatorGamepad.getPovIfAvailable(0).onTrue(removeHighAlgae);

        var collectGroundAlgae = prepAlgaeSystemCommandGroupFactory.create(AlgaeArmSubsystem.AlgaeArmPositions.GroundCollection);
        oi.operatorGamepad.getPovIfAvailable(270).onTrue(collectGroundAlgae);

        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper).whileTrue(intakeAlgae);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).whileTrue(ejectAlgae);

        var homeAlgaeArm = setAlgaeArmProvider.get();
        homeAlgaeArm.setTargetPosition(AlgaeArmSubsystem.AlgaeArmPositions.FullyRetracted);
        oi.operatorGamepad.getPovIfAvailable(90).onTrue(homeAlgaeArm);
    }

    @Inject
    public void setupDriverStationDashboardCommands(CalibrateDriveCommand calibrateDriveCommand) {
        calibrateDriveCommand.includeOnSmartDashboard();
    }

    // Programmer commands are only meant to be used to debug or test the robot. They should not be used in competition,
    // and many do dangerous things like bypass various safeties or force the robot into states that aren't useful
    // (e.g. only driving a single swerve module at a time for testing purposes).
    @Inject
    public void setupSuperstructureGamepadCommands(
            OperatorInterface oi,
            IntakeCoralCommand intakeCoralCommand,
            ScoreCoralCommand scoreCoralCommand,
            Provider<SetCoralArmTargetAngleCommand> setArmTargetAngleCommandProvider,
            Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider,
            ForceElevatorCalibratedCommand forceElevatorCalibratedCommand,
            ForceCoralArmCalibratedCommand forceCoralArmCalibratedCommand) {

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

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.LeftTrigger).whileTrue(intakeCoralCommand);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.RightTrigger).whileTrue(scoreCoralCommand);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper).onTrue(lowerToHumanLoad);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).onTrue(riseToScore);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Start).onTrue(forceElevatorCalibratedCommand);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.A).whileTrue(riseToL2);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.B).whileTrue(riseToL3);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(riseToL4);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Back).onTrue(forceCoralArmCalibratedCommand);



    }

    @Inject
    public void setUpButtonCommands(OperatorInterface oi, CoralArmSubsystem coralArmSubsystem) {
        oi.neoTrellis.getifAvailable(9)
                .onTrue(coralArmSubsystem.createSetTargetCoralLevelCommand(Landmarks.CoralLevel.TWO));
        oi.neoTrellis.getifAvailable(10)
                .onTrue(coralArmSubsystem.createSetTargetCoralLevelCommand(Landmarks.CoralLevel.THREE));
        oi.neoTrellis.getifAvailable(11)
                .onTrue(coralArmSubsystem.createSetTargetCoralLevelCommand(Landmarks.CoralLevel.FOUR));
    }

    @Inject
    public void setupSysIdCommands(

            DriveSubsystem drive,
            ElevatorSubsystem elevator
    ) {
/*
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
*/
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