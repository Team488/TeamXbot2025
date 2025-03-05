package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.DriveToStationAndIntakeUntilCollectedCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;

public class BlueMidCagePushRobotLevelFoursAuto extends BaseAutonomousSequentialCommandGroup {

    @Inject
    public BlueMidCagePushRobotLevelFoursAuto(AutonomousCommandSelector autoSelector,
                                              PoseSubsystem pose,
                                              Provider<DriveToFaceAndScoreCommandGroupFactory> driveToFaceAndScoreFactProv,
                                              Provider<DriveToStationAndIntakeUntilCollectedCommandGroupFactory> driveToStationAndIntakeFactProv,
                                              PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact,
                                              BaseSimulator simulator) {
        super(autoSelector);

        Pose2d midBlueCage = Landmarks.BlueCageTwoStartingLine;
        Pose2d midBlueCageHalfRobotBackPose = PoseSubsystem.convertBlueToRedIfNeeded(
                new Pose2d(midBlueCage.getX() + 0.5, midBlueCage.getY(), Rotation2d.fromDegrees(180)));

        var startInFrontOfCageHalfRobotBack = pose.createSetPositionCommand(
                PoseSubsystem.convertBlueToRedIfNeeded(midBlueCageHalfRobotBackPose));
        this.addCommands(startInFrontOfCageHalfRobotBack);

        var resetSim = new InstantCommand(() -> simulator.resetPosition(
                PoseSubsystem.convertBlueToRedIfNeeded(midBlueCageHalfRobotBackPose)));
        this.addCommands(resetSim);

        // Drive to far left, branch B and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        // Drive to left coral station and intake coral until collected
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID);
        var driveToLeftStationAndIntakeFirst = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, true);
        this.addCommands(driveToLeftStationAndIntakeFirst);

        // Drive to close left, branch B and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreCloseLeftBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Drive to left coral station and intake coral until collected
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID);
        var driveToLeftStationAndIntakeSecond = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, false);
        this.addCommands(driveToLeftStationAndIntakeSecond);

        // Drive to close left, branch A and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        var driveAndScoreCloseLeftBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Drive to left coral station and intake coral until collected
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID);
        var driveToLeftStationAndIntakeThird = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, false);
        this.addCommands(driveToLeftStationAndIntakeThird);

        // Drive to close, branch A and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        // TODO: replace this with drive and intake commandgroup instead
        var homeCoralSystem = prepCoralSystemCommandGroupFact.create(() -> Landmarks.CoralLevel.COLLECTING);
        this.addCommands(homeCoralSystem);
    }
}

