package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.DriveToStationAndIntakeUntilCollectedCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;

public class FromRightCageScoreRightFacesLevelFours extends BaseAutonomousSequentialCommandGroup {

    @Inject
    public FromRightCageScoreRightFacesLevelFours(AutonomousCommandSelector autoSelector,
                                                  PoseSubsystem pose,
                                                  Provider<DriveToFaceAndScoreCommandGroupFactory> driveToFaceAndScoreFactProv,
                                                  Provider<DriveToStationAndIntakeUntilCollectedCommandGroupFactory> driveToStationAndIntakeFactProv,
                                                  PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact,
                                                  BaseSimulator simulator) {
        super(autoSelector);

        // Drive to far Right, branch B and score level four
        var driveAndScoreFarRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarRightBranchBLevelFour);

        // Drive to right coral station, far section and intake coral until collected
        var driveToRightStationAndIntakeFirst = driveToStationAndIntakeFactProv.get().create(
                        Landmarks.CoralStation.RIGHT, true)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.RIGHT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeFirst);

        // Drive to close right, branch B and score level four
        var driveAndScoreCloseRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseRightBranchBLevelFour);

        // Drive to right coral station, close section and intake coral until collected
        var driveToRightStationAndIntakeSecond = driveToStationAndIntakeFactProv.get().create(
                        Landmarks.CoralStation.RIGHT, false)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.RIGHT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeSecond);

        // Drive to close right, branch A and score level four
        var driveAndScoreCloseRightBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseRightBranchALevelFour);

        // Drive to right coral station, far section and intake coral until collected again
        var driveToRightStationAndIntakeThird = driveToStationAndIntakeFactProv.get().create(
                        Landmarks.CoralStation.RIGHT, false)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.RIGHT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeThird);

        // Drive to close, branch A and score level four
        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.CLOSE, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        // TODO: replace this with drive and intake commandgroup instead
        var homeCoralSystem = prepCoralSystemCommandGroupFact.create(() -> Landmarks.CoralLevel.CORAL_COLLECTING);
        this.addCommands(homeCoralSystem);
    }
}
