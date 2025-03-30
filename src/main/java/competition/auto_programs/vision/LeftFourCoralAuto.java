package competition.auto_programs.vision;

import static competition.subsystems.pose.vision.Paths.blueCloseBranchAToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.blueCloseLeftBranchAToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.blueCloseLeftBranchBToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.farLeftBToCoralStationBlue;

import competition.auto_programs.BaseAutonomousSequentialCommandGroup;
import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.vision_path.PathDriveToLocationForCoralStationFactory;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.vision.Paths;

import javax.inject.Inject;
import javax.inject.Provider;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

public class LeftFourCoralAuto extends BaseAutonomousSequentialCommandGroup {

    @Inject
    public LeftFourCoralAuto(
            AutonomousCommandSelector autoSelector,
            DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
            Provider<PathDriveToLocationForCoralStationFactory> pathDriveToLocationAndIntakeUntilCollectedProvider) {
        super(autoSelector);

        // When we see loop overruns at the start of auto, adding a wait seemed to help
        // this.addCommands(new WaitCommand(1.0));

        // Score 1
        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT,
                Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        // Coral Station
        var driveToLeftStation =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.LEFT, farLeftBToCoralStationBlue)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.LEFT,
                                Landmarks.CoralStationSection.MID));

        this.addCommands(driveToLeftStation);

        // Score 2
        var driveAndScoreCloseLeftBranchBLevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Coral Station
        var driveToRightStationAndIntakeSecond =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.LEFT, blueCloseLeftBranchBToLeftCoralStation)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.LEFT,
                                Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeSecond);

        // Score 3
        var driveAndScoreCloseLeftBranchALevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Coral Station
        var driveToLeftStationAndIntakeSecond =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.LEFT, blueCloseLeftBranchAToLeftCoralStation)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.LEFT,
                                Landmarks.CoralStationSection.MID));
        this.addCommands(driveToLeftStationAndIntakeSecond);

        // Score 4
        var driveAndScoreCloseBranchALevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(
                                getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE,
                                        Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        // Back to coral station
        var homed = pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                .create(Paths.Side.LEFT, blueCloseBranchAToLeftCoralStation)
                .alongWith(getDriveAndIntakeStatusMessageCommand(
                        Landmarks.CoralStation.LEFT,
                        Landmarks.CoralStationSection.MID));
        this.addCommands(homed);
    }
}