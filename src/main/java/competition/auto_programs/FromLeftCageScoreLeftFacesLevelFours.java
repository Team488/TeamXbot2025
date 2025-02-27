package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.DriveToStationAndIntakeUntilCollectedCommandGroupFactory;
import competition.simulation.MapleSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.apache.logging.log4j.core.time.Instant;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;

public class FromLeftCageScoreLeftFacesLevelFours extends BaseAutonomousSequentialCommandGroup {

    @Inject
    public FromLeftCageScoreLeftFacesLevelFours(AutonomousCommandSelector autoSelector,
                                                PoseSubsystem pose,
                                                DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
                                                DriveToStationAndIntakeUntilCollectedCommandGroupFactory driveToStationAndIntakeFact,
                                                Provider<DriveToFaceAndScoreCommandGroupFactory> driveToFaceAndScoreFactProv,
                                                Provider<DriveToStationAndIntakeUntilCollectedCommandGroupFactory> driveToStationAndIntakeFactProv,
                                                MapleSimulator mapleSimulator) {
        super(autoSelector);

        // Force our location to start in front of left cage
        var startInFrontOfLeftCage = pose.createSetPositionCommand(
                () -> PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueLeftStartingLine)
        );
        this.addCommands(startInFrontOfLeftCage);

        var resetMapleSim = new InstantCommand(() -> mapleSimulator.resetPosition(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueLeftStartingLine)));
        this.addCommands(resetMapleSim);

        // Drive to far left, branch A and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreFarLeftBranchALevelFour);

        // Drive to left coral station, far section and intake coral until collected
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR);
        var driveToLeftStationFarSectionAndIntake = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR);
        this.addCommands(driveToLeftStationFarSectionAndIntake);

        // Drive to close left, branch A and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        var driveAndScoreCloseLeftBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Drive to left coral station, close section and intake coral until collected
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.CLOSE);
        var driveToLeftStationCloseSectionAndIntake = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.CLOSE);
        this.addCommands(driveToLeftStationCloseSectionAndIntake);

        // Drive to close left, branch B and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreCloseLeftBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Drive to left coral station, far section and intake coral until collected again
        queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR);
        var driveToLeftStationFarSectionAndIntakeSecond = driveToStationAndIntakeFactProv.get().create(
                Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.FAR);
        this.addCommands(driveToLeftStationFarSectionAndIntakeSecond);

        // Drive to far left, branch B and score level four
        queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);
    }
}
