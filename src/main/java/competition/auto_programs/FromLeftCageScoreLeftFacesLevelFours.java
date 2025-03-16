package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.DriveToStationAndIntakeUntilCollectedCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.commandgroups.vision_path.PathToNearestStationAndIntakeUntilCollectedCommandGroupFactory;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;

public class FromLeftCageScoreLeftFacesLevelFours extends BaseAutonomousSequentialCommandGroup {

    @Inject
    public FromLeftCageScoreLeftFacesLevelFours(AutonomousCommandSelector autoSelector,
                                                PoseSubsystem pose,
                                                DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
                                                DriveToStationAndIntakeUntilCollectedCommandGroupFactory driveToStationAndIntakeFact,
                                                PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact,
                                                BaseSimulator simulator,
                                                PathToNearestStationAndIntakeUntilCollectedCommandGroupFactory
                                                            pathToNearestStationAndIntakeFactory) {
        super(autoSelector);

        // Drive to far left, branch B and score level four
        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        // Drive to left coral station and intake coral until collected
        var driveToLeftStationAndIntakeFirst = driveToStationAndIntakeFact.create(
                        Landmarks.CoralStation.LEFT, true)
                .alongWith(getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
//        this.addCommands(driveToLeftStationAndIntakeFirst);

        var pathToLeftStationAndIntakeFirst = pathToNearestStationAndIntakeFactory.create(true);
        this.addCommands(pathToLeftStationAndIntakeFirst);

        // Drive to close left, branch B and score level four
        var driveAndScoreCloseLeftBranchBLevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Drive to left coral station and intake coral until collected
        var driveToLeftStationAndIntakeSecond = driveToStationAndIntakeFact.create(
                        Landmarks.CoralStation.LEFT, false)
                .alongWith(getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
//        this.addCommands(driveToLeftStationAndIntakeSecond);

        var pathToLeftStationAndIntakeSecond = pathToNearestStationAndIntakeFactory.create(true);
        this.addCommands(pathToLeftStationAndIntakeSecond);

        // Drive to close left, branch A and score level four
        var driveAndScoreCloseLeftBranchALevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Drive to left coral station and intake coral until collected
        var driveToLeftStationAndIntakeThird = driveToStationAndIntakeFact.create(
                        Landmarks.CoralStation.LEFT, false)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToLeftStationAndIntakeThird);

        // Drive to close, branch A and score level four
        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        // TODO: replace this with drive and intake commandgroup instead
        var homeCoralSystem = prepCoralSystemCommandGroupFact.create(() -> Landmarks.CoralLevel.COLLECTING);
        this.addCommands(homeCoralSystem);
    }
}
