package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.DriveToStationAndIntakeUntilCollectedCommandGroupFactory;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;

public class FromLeftCageScoreLeftFacesLevelFours extends SequentialCommandGroup {

    final AutonomousCommandSelector autoSelector;

    @Inject
    public FromLeftCageScoreLeftFacesLevelFours(AutonomousCommandSelector autoSelector,
                                                PoseSubsystem pose,
                                                DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
                                                DriveToStationAndIntakeUntilCollectedCommandGroupFactory driveToStationAndIntakeFact,
                                                Provider<DriveToFaceAndScoreCommandGroupFactory> driveToFaceAndScoreFactProv,
                                                Provider<DriveToStationAndIntakeUntilCollectedCommandGroupFactory> driveToStationAndIntakeFactProv) {
        this.autoSelector = autoSelector;

        // Force our location to start in front of left cage
        var startInFrontOfLeftCage = pose.createSetPositionCommand(
                () -> Landmarks.BlueLeftStartingLine
        );
        this.addCommands(startInFrontOfLeftCage);

        // Drive to far left, branch A and score level four
        queueMessageToAutoSelector("Drive to far left, branch A and score level four");
        var driveAndScoreFarLeftBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreFarLeftBranchALevelFour);

        // Drive to left coral station, far section and intake coral until collected
        queueMessageToAutoSelector("Drive to left coral station, far section and intake coral until collected");
        var driveToLeftStationFarSectionAndIntake = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR);
        this.addCommands(driveToLeftStationFarSectionAndIntake);

        // Drive to close left, branch A and score level four
        queueMessageToAutoSelector("Drive to close left, branch A and score level four");
        var driveAndScoreCloseLeftBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Drive to left coral station, close section and intake coral until collected
        queueMessageToAutoSelector("Drive to left coral station, close section and intake coral until collected");
        var driveToLeftStationCloseSectionAndIntake = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.CLOSE);
        this.addCommands(driveToLeftStationCloseSectionAndIntake);

        // Drive to close left, branch B and score level four
        queueMessageToAutoSelector("Drive to close left, branch B and score level four");
        var driveAndScoreCloseLeftBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Drive to left coral station, far section and intake coral until collected again
        queueMessageToAutoSelector("Drive to left coral station, far section and intake coral until collected again");
        var driveToLeftStationFarSectionAndIntakeSecond = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR);
        this.addCommands(driveToLeftStationFarSectionAndIntakeSecond);

        // Drive to far left, branch B and score level four
        queueMessageToAutoSelector("Drive to far left, branch B and score level four");
        var driveAndScoreFarLeftBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);
    }

    private void queueMessageToAutoSelector(String message) {
        this.addCommands(autoSelector.createAutonomousStateMessageCommand(message));
    }
}
