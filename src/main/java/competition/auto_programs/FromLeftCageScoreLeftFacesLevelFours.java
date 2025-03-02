package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.DriveToStationAndIntakeUntilCollectedCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.simulation.BaseSimulator;
import competition.simulation.MapleSimulator;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.drive.commands.AlignToHLSAndIntakeUntilCollectedCommandGroupFactory;
import competition.subsystems.drive.commands.AlignToSpecificHumanLoadingStationCommand;
import competition.subsystems.drive.commands.DriveToCoralStationSectionCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;

public class FromLeftCageScoreLeftFacesLevelFours extends BaseAutonomousSequentialCommandGroup {
    double timeOutSeconds = 8;
    CoralScorerSubsystem coralScorerSubsystem;

    @Inject
    public FromLeftCageScoreLeftFacesLevelFours(AutonomousCommandSelector autoSelector,
                                                PoseSubsystem pose,
                                                DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
                                                DriveToStationAndIntakeUntilCollectedCommandGroupFactory driveToStationAndIntakeFact,
                                                Provider<DriveToFaceAndScoreCommandGroupFactory> driveToFaceAndScoreFactProv,
                                                Provider<DriveToStationAndIntakeUntilCollectedCommandGroupFactory> driveToStationAndIntakeFactProv,
                                                PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact,
                                                Provider<AlignToHLSAndIntakeUntilCollectedCommandGroupFactory> alignToHLSAndIntakeUntilCollectedCommandGroupFactoryProvider,
                                                CoralScorerSubsystem coralScorerSubsystem,
                                                BaseSimulator simulator,
                                                DriveToCoralStationSectionCommand driveToCoralStationSectionCommand) {
        super(autoSelector);
        this.coralScorerSubsystem = coralScorerSubsystem;

        // Force our location to start in front of left cage
        var startInFrontOfLeftCage = pose.createSetPositionCommand(
                () -> PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCageTwoStartingLine)
        );
        this.addCommands(startInFrontOfLeftCage);

        var resetSim = new InstantCommand(() -> simulator.resetPosition(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCageTwoStartingLine)));
        this.addCommands(resetSim);

        // Drive to far left, branch B and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        // Drive to left coral station, far section and intake coral until collected
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR);
        var driveToLeftStationFarSectionAndIntake = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR, true);
        this.addCommands(driveToLeftStationFarSectionAndIntake.withTimeout(timeOutSeconds));

        var firstAlignToCoralStation = alignToHLSAndIntakeUntilCollectedCommandGroupFactoryProvider
                .get().create(Landmarks.CoralStation.LEFT);
        var goToCoralStationIntakeUntilCollected = new ParallelDeadlineGroup(firstAlignToCoralStation);
        var firstTryLoadingAgain = new ConditionalCommand(new WaitCommand(0), goToCoralStationIntakeUntilCollected,
                () -> coralScorerSubsystem.confidentlyHasCoral());
        this.addCommands(firstTryLoadingAgain);

        // Drive to close left, branch B and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreCloseLeftBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Drive to left coral station, close section and intake coral until collected
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.CLOSE);
        var driveToLeftStationCloseSectionAndIntake = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID, false);
        this.addCommands(driveToLeftStationCloseSectionAndIntake);

        // Drive to close left, branch A and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        var driveAndScoreCloseLeftBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Drive to left coral station, far section and intake coral until collected again
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR);
        var driveToLeftStationFarSectionAndIntakeSecond = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.CLOSE, false);
        this.addCommands(driveToLeftStationFarSectionAndIntakeSecond);

        // Drive to close, branch A and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        var homeCoralSystem = prepCoralSystemCommandGroupFact.create(() -> Landmarks.CoralLevel.COLLECTING);
        this.addCommands(homeCoralSystem);
    }
}
