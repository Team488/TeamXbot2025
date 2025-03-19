package competition.auto_programs.vision;

import competition.auto_programs.BaseAutonomousSequentialCommandGroup;
import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.commandgroups.vision_path.PathDriveToCoralStationAndIntakeUntilCollected;
import competition.commandgroups.vision_path.PathDriveToLocationAndIntakeUntilCollected;
import competition.commandgroups.vision_path.PathToFaceAndScoreCommandGroupFactory;
import competition.electrical_contract.ElectricalContract;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;

import static competition.subsystems.pose.vision.Paths.blueCloseBranchAToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.blueCloseLeftBranchAToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.blueCloseLeftBranchBToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.farLeftBToCoralStationBlue;


public class LeftFourCoralAuto extends BaseAutonomousSequentialCommandGroup {
    @Inject
    public LeftFourCoralAuto(
            AprilTagFieldLayout aprilTagFieldLayout,
            ElectricalContract electricalContract,
            AutonomousCommandSelector autoSelector,
            CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem,
            PoseSubsystem pose,
            Provider<PathDriveToCoralStationAndIntakeUntilCollected> driveToStationAndIntakeFactProv,
            DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
            Provider<PathToFaceAndScoreCommandGroupFactory> driveToFaceAndScoreFactProv,
            Provider<PathDriveToLocationAndIntakeUntilCollected> pathDriveToLocationAndIntakeUntilCollectedProvider,
            PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact,
            BaseSimulator simulator) {

        super(autoSelector);
        var initializeStateCommand = pose.createSetPositionCommand(
                        () -> PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCageOneStartingLine)
                )
                .alongWith(new InstantCommand(() -> simulator.resetPosition(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCageOneStartingLine))));
        this.addCommands(initializeStateCommand);
        // Score 1

//        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
//        var driveAndScoreFarRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR, blueCageOneStartingLineToFarLeftBranchA)
//                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreFarRightBranchBLevelFour);
        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        // Coral Station
        var driveToLeftStation = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(farLeftBToCoralStationBlue)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToLeftStation);

        // Score 2
//        var driveAndScoreCloseRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR, blueLeftCoralStationToCloseLeftBranchA)
//                .alongWith(
//                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreCloseRightBranchBLevelFour);
        var driveAndScoreCloseLeftBranchBLevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Coral Station
        var driveToRightStationAndIntakeSecond = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(blueCloseLeftBranchBToLeftCoralStation)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeSecond);


        // Score 3
//        var driveAndScoreCloseaBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR, blueLeftCoralStationToCloseLeftBranchA)
//                .alongWith(
//                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreCloseaBranchALevelFour);
        var driveAndScoreCloseLeftBranchALevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Coral Station
        var driveToLeftStationAndIntakeSecond = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(blueCloseLeftBranchAToLeftCoralStation)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToLeftStationAndIntakeSecond);

        // Score 4
//        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR, blueLeftCoralStationToCloseBranchA)
//                .alongWith(
//                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreCloseBranchALevelFour);
        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        // Back to coral station
        var homed = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(blueCloseBranchAToLeftCoralStation)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(homed);
    }
}
