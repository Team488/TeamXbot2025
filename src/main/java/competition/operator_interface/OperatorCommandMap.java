package competition.operator_interface;

import competition.auto_programs.FromCageScoreOneCoralAutoFactory;
import competition.auto_programs.FromLeftCageScoreLeftFacesLevelFours;
import competition.auto_programs.FromRightCageScoreRightFacesLevelFours;
import competition.auto_programs.vision.LeftFourCoralAuto;
import competition.auto_programs.vision.RightFourCoralAuto;
import competition.commandgroups.DriveToClosestStationCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.commandgroups.vision_path.PathDriveToLocationForCoralStationFactory;
import competition.simulation.commands.ResetSimulatedPose;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_arm.commands.ForceCoralArmCalibratedCommand;
import competition.subsystems.coral_arm.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.coral_scorer.commands.IntakeAlgaeCommand;
import competition.subsystems.coral_scorer.commands.IntakeCoralCommand;
import competition.subsystems.coral_scorer.commands.ScoreAlgaeCommand;
import competition.subsystems.coral_scorer.commands.ScoreCoralCommand;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.CalibrateDriveCommand;
import competition.subsystems.drive.commands.DebugSwerveModuleCommand;
import competition.subsystems.drive.commands.AlignToNearestReefFaceForAlgaeCommand;
import competition.subsystems.drive.commands.DriveToBargeCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToBargeCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.ForceElevatorCalibratedCommand;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import competition.subsystems.elevator.commands.ToggleElevatorMotionMagicCommand;
import competition.subsystems.elevator.commands.TrimElevatorDown;
import competition.subsystems.elevator.commands.TrimElevatorUp;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.subsystems.autonomous.SetAutonomousCommand;
import xbot.common.subsystems.drive.swerve.commands.ChangeActiveSwerveModuleCommand;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;

/**
 * Maps operator interface buttons to commands
 */
@Singleton
public class OperatorCommandMap {

    @Inject
    public OperatorCommandMap() {
    }

    @Inject
    public void setupDriverCommands(
            OperatorInterface operatorInterface,
            SetRobotHeadingCommand resetHeading,
            Provider<AlignToReefWithAprilTagCommand> alignToReefWithAprilTagProvider,
            DebugSwerveModuleCommand debugModule,
            ChangeActiveSwerveModuleCommand changeActiveModule,
            SwerveDriveWithJoysticksCommand typicalSwerveDrive,
            AlignToNearestReefFaceForAlgaeCommand alignToNearestReefFaceForAlgaeCommand,
            DriveSubsystem drive, PoseSubsystem pose,
            DriveToClosestStationCommandGroupFactory
                    driveToClosestStationCommandGroupFactory,
            CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem,
            PathDriveToLocationForCoralStationFactory pathDriveToLocationForCoralStationFactory,
            AlignCameraToAprilTagCalculator.AlignCameraToAprilTagCalculatorFactory aprilTagCalculatorFactory,
            PathDriveToBargeCommand pathDriveToBargeCommand,
            DriveToBargeCommand driveToBargeCommand) {
        resetHeading.setHeadingToApply(0);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.Start).onTrue(resetHeading);

        var pointAtNearestCoralStation = drive.createSetDynamicHeadingTargetCommand(() ->
                PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.getCoralStationSectionPose(pose.getClosestCoralStation(), Landmarks.CoralStationSection.MID)
                        .getRotation()));
        var clearPointAtHeading = drive.createClearAllHeadingTargetsCommand();

        double alignToCoralOffsetInches = -2;
        var alignToReefWithAprilTagWithLeftCamera = alignToReefWithAprilTagProvider.get();
        alignToReefWithAprilTagWithLeftCamera.setConfigurations(
                Cameras.FRONT_LEFT_CAMERA.getIndex(), false, alignToCoralOffsetInches, true,
                AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, false);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).whileTrue(alignToReefWithAprilTagWithLeftCamera);

        var alignToReefWithAprilTagWithRightCamera = alignToReefWithAprilTagProvider.get();
        alignToReefWithAprilTagWithRightCamera.setConfigurations(
                Cameras.FRONT_RIGHT_CAMERA.getIndex(), false, alignToCoralOffsetInches, true,
                AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, false);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper).whileTrue(alignToReefWithAprilTagWithRightCamera);

        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.Y).whileTrue(pointAtNearestCoralStation)
                .onFalse(clearPointAtHeading);
        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(alignToNearestReefFaceForAlgaeCommand);


        // Instantly drives to closest coral station
//        SequentialCommandGroup pathDriveToClosestCoralStation = pathDriveToLocationForCoralStationFactory.createDriveOnly(
//                null, null
//        );
//        SequentialCommandGroup driveToClosestCoralStation =
//                driveToClosestStationCommandGroupFactory.createDriveOnly();
//        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.A).whileTrue(new ConditionalCommand(
//                pathDriveToClosestCoralStation,
//                driveToClosestCoralStation,
//                () -> coprocessorCommunicationSubsystem.isCoralStationPathConfident(pose)
//        ));



//        operatorInterface.driverGamepad.getifAvailable(XXboxController.XboxButton.B).whileTrue(new ConditionalCommand(
//                pathDriveToBargeCommand,
//                driveToBargeCommand,
//                () -> coprocessorCommunicationSubsystem.isBargePathConfident(pose)
//        ));


//        operatorInterface.driverGamepad.getPovIfAvailable(0).onTrue(debugModule);
//        operatorInterface.driverGamepad.getPovIfAvailable(90).onTrue(changeActiveModule);
//        operatorInterface.driverGamepad.getPovIfAvailable(180).onTrue(typicalSwerveDrive);

        var aprilTagCalculator = aprilTagCalculatorFactory.create();

        operatorInterface.driverGamepad.getPovIfAvailable(90).onTrue(aprilTagCalculator.createDecreaseOffsetByOneInchCommand());
        operatorInterface.driverGamepad.getPovIfAvailable(270).onTrue(aprilTagCalculator.createIncreaseOffsetByOneInchCommand());
    }


    @Inject
    public void setUpOperatorCommands(OperatorInterface oi,
                                      PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                      ScoreCoralCommand scoreCoralCommand,
                                      ScoreWhenReadyCommand scoreWhenReadyCommand,
                                      ForceElevatorCalibratedCommand forceElevatorCalibratedCommand,
                                      ForceCoralArmCalibratedCommand forceCoralPivotCalibratedCommand,
                                      Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider,
                                      CoralArmSubsystem coralArmSubsystem,
                                      IntakeCoralCommand intakeCoralCommand,
                                      IntakeAlgaeCommand intakeAlgaeCommand,
                                      ScoreAlgaeCommand scoreAlgaeCommand) {
        // Coral system buttons
        var prepL4 = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.FOUR);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.Y).onTrue(prepL4);

        var prepL3 = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.THREE);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(prepL3);

        var prepL2 = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.TWO);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.A).onTrue(prepL2);

        var homed = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.CORAL_COLLECTING);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(homed);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.LeftTrigger).whileTrue(intakeCoralCommand);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.RightTrigger).whileTrue(scoreCoralCommand);

        var calibrateSuperstructure = Commands.parallel(
                forceElevatorCalibratedCommand,
                forceCoralPivotCalibratedCommand
        ).ignoringDisable(true);


        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.Start).onTrue(calibrateSuperstructure);
        // Algae system buttons

        var removeLowAlgae = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.LOW_ALGAE);
        oi.operatorGamepad.getPovIfAvailable(180).onTrue(removeLowAlgae);

        var removeHighAlgae = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.HIGH_ALGAE);
        oi.operatorGamepad.getPovIfAvailable(0).onTrue(removeHighAlgae);

//        var scoreAlgaeInNetHeight = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.SCORE_ALGAE_NET);
//        oi.operatorGamepad.getPovIfAvailable(90).onTrue(scoreAlgaeInNetHeight);
//
//        var scoreAlgaeInProcessor = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.SCORE_ALGAE_PROCESSOR);
//        oi.operatorGamepad.getPovIfAvailable(270).onTrue(scoreAlgaeInProcessor);

        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper).whileTrue(intakeAlgaeCommand);
        oi.operatorGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).whileTrue(scoreAlgaeCommand);
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
            ForceCoralArmCalibratedCommand forceCoralArmCalibratedCommand,

            ToggleElevatorMotionMagicCommand toggleElevatorMotionMagicCommand) {

        var returnToBase = setElevatorTargetHeightCommandProvider.get();
        returnToBase.setHeight(Landmarks.CoralLevel.CORAL_COLLECTING);
        var riseToL2 = setElevatorTargetHeightCommandProvider.get();
        riseToL2.setHeight(Landmarks.CoralLevel.TWO);
        var riseToL3 = setElevatorTargetHeightCommandProvider.get();
        riseToL3.setHeight(Landmarks.CoralLevel.THREE);
        var riseToL4 = setElevatorTargetHeightCommandProvider.get();
        riseToL4.setHeight(Landmarks.CoralLevel.FOUR);

        var riseToScore = setArmTargetAngleCommandProvider.get();
        riseToScore.setAngle(Landmarks.CoralLevel.FOUR);
        var lowerToHumanLoad = setArmTargetAngleCommandProvider.get();
        lowerToHumanLoad.setAngle(Landmarks.CoralLevel.CORAL_COLLECTING);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.LeftTrigger).whileTrue(intakeCoralCommand);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.RightTrigger).whileTrue(scoreCoralCommand);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.LeftBumper).onTrue(lowerToHumanLoad);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.RightBumper).onTrue(riseToScore);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Start).onTrue(forceElevatorCalibratedCommand);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.A).whileTrue(toggleElevatorMotionMagicCommand);
//        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Y).whileTrue(returnToBase);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.B).whileTrue(riseToL3);
        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.X).whileTrue(riseToL4);

        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.Back).onTrue(forceCoralArmCalibratedCommand);

//        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.X).onTrue(repositionAlgaeArmUp);
//        oi.superstructureGamepad.getifAvailable(XXboxController.XboxButton.B).onTrue(repositionAlgaeArmDown);

    }

    @Inject
    public void setupNeoTrellis(OperatorInterface oi, CoralArmSubsystem coralArmSubsystem, TrimElevatorUp trimElevatorUp,
                                TrimElevatorDown trimElevatorDown) {
        oi.neoTrellis.getifAvailable(9)
                .onTrue(coralArmSubsystem.createSetTargetCoralLevelCommand(Landmarks.CoralLevel.TWO));
        oi.neoTrellis.getifAvailable(10)
                .onTrue(coralArmSubsystem.createSetTargetCoralLevelCommand(Landmarks.CoralLevel.THREE));
        oi.neoTrellis.getifAvailable(11)
                .onTrue(coralArmSubsystem.createSetTargetCoralLevelCommand(Landmarks.CoralLevel.FOUR));
        oi.neoTrellis.getifAvailable(8).onTrue(trimElevatorUp);
        oi.neoTrellis.getifAvailable(16).onTrue(trimElevatorDown);
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
    public void setupAutonomousCommands(OperatorInterface oi,
                                        Provider<SetAutonomousCommand> setAutonomousCommandProvider,
                                        Provider<FromCageScoreOneCoralAutoFactory> fromCageScoreOneLevelFourAutoFactProv,
                                        FromLeftCageScoreLeftFacesLevelFours fromLeftCageScoreLeftFacesLevelFours,
                                        FromRightCageScoreRightFacesLevelFours fromRightCageScoreRightFacesLevelFours,
                                        Provider<LeftFourCoralAuto> leftFourCoralAutoProvider,
                                        Provider<RightFourCoralAuto> rightFourCoralAuto
    ) {
        var setFromLeftFarLeftBranchBLevelFour = setAutonomousCommandProvider.get();
        setFromLeftFarLeftBranchBLevelFour.setAutoCommand(fromCageScoreOneLevelFourAutoFactProv.get().create(
                Landmarks.BlueCageOneStartingLine, Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR
        ));
        oi.neoTrellis.getifAvailable(1).onTrue(setFromLeftFarLeftBranchBLevelFour); // temporary button
        setFromLeftFarLeftBranchBLevelFour.includeOnSmartDashboard("From Left Score Far Left Branch B Level 4 Auto");

        var setFromMidFarBranchBLevelFour = setAutonomousCommandProvider.get();
        setFromMidFarBranchBLevelFour.setAutoCommand(fromCageScoreOneLevelFourAutoFactProv.get().create(
                Landmarks.BlueMidOfLine, Landmarks.ReefFace.FAR, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR
        ));
        oi.neoTrellis.getifAvailable(2).onTrue(setFromMidFarBranchBLevelFour); // temporary button
        setFromMidFarBranchBLevelFour.includeOnSmartDashboard("From Mid Score Far Branch B Level 4 Auto");

        var setFromRightFarRightBranchALevelFour = setAutonomousCommandProvider.get();
        setFromRightFarRightBranchALevelFour.setAutoCommand(fromCageScoreOneLevelFourAutoFactProv.get().create(
                Landmarks.BlueCageSixStartingLine, Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR
        ));
        oi.neoTrellis.getifAvailable(3).onTrue(setFromRightFarRightBranchALevelFour); // temporary button
        setFromRightFarRightBranchALevelFour.includeOnSmartDashboard("From Right Score Far Right Branch A Level 4 Auto");

        var setFromLeftCageScoreLeftFacesLevelFours = setAutonomousCommandProvider.get();
        setFromLeftCageScoreLeftFacesLevelFours.setAutoCommand(fromLeftCageScoreLeftFacesLevelFours, Landmarks.BlueCageOneStartingLine);
        oi.neoTrellis.getifAvailable(4).onTrue(setFromLeftCageScoreLeftFacesLevelFours); // temporary button
        setFromLeftCageScoreLeftFacesLevelFours.includeOnSmartDashboard("From Left Score Left Face Level Fours Auto");

        var setFromRightCageScoreRightFacesLevelFours = setAutonomousCommandProvider.get();
        setFromRightCageScoreRightFacesLevelFours.setAutoCommand(fromRightCageScoreRightFacesLevelFours, Landmarks.BlueCageSixStartingLine);
        oi.neoTrellis.getifAvailable(5).onTrue(setFromRightCageScoreRightFacesLevelFours);
        setFromRightCageScoreRightFacesLevelFours.includeOnSmartDashboard("From Right Score Right Face Level Fours auto");

        var leftVisionAuto = setAutonomousCommandProvider.get();
        leftVisionAuto.setAutoCommand(leftFourCoralAutoProvider.get(), Landmarks.BlueCageOneStartingLine);
        leftVisionAuto.includeOnSmartDashboard("Left Vision Auto");
        
        var rightVisionAuto = setAutonomousCommandProvider.get();
        rightVisionAuto.setAutoCommand(rightFourCoralAuto.get(), Landmarks.BlueCageSixStartingLine);
        rightVisionAuto.includeOnSmartDashboard("Right Vision Auto");
    }

    @Inject
    public void setupSimulatorCommands(
            ResetSimulatedPose resetPose
    ) {
        resetPose.includeOnSmartDashboard();
    }
}