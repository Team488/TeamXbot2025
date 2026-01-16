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

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

public class RightFourCoralAuto extends BaseAutonomousSequentialCommandGroup {

    @Inject
    public RightFourCoralAuto(
            AutonomousCommandSelector autoSelector,
            DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
            Provider<PathDriveToLocationForCoralStationFactory> pathDriveToLocationAndIntakeUntilCollectedProvider) {
        super(autoSelector);

        this.addCommands(new WaitCommand(Seconds.of(1)));

        // Score 1
        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_RIGHT,
                Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        // Coral Station
        var driveToLeftStation =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.RIGHT, farLeftBToCoralStationBlue)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.RIGHT,
                                Landmarks.CoralStationSection.MID));

        this.addCommands(driveToLeftStation);

        // Score 2
        var driveAndScoreCloseLeftBranchBLevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Coral Station
        var driveToRightStationAndIntakeSecond =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.RIGHT, blueCloseLeftBranchBToLeftCoralStation)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.RIGHT,
                                Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeSecond);

        // Score 3
        var driveAndScoreCloseLeftBranchALevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Coral Station
        var driveToLeftStationAndIntakeSecond =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.RIGHT, blueCloseLeftBranchAToLeftCoralStation)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.RIGHT,
                                Landmarks.CoralStationSection.MID));
        this.addCommands(driveToLeftStationAndIntakeSecond);

        // Score 4
        var driveAndScoreCloseBranchALevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(
                                getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE,
                                        Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        // Back to coral station
        var homed =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.RIGHT, blueCloseBranchAToLeftCoralStation)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.RIGHT,
                                Landmarks.CoralStationSection.MID));
        this.addCommands(homed);
    }
}