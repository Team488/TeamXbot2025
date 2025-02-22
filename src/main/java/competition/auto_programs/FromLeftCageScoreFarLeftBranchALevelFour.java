package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.simulation.MapleSimulator;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import java.util.Set;

public class FromLeftCageScoreFarLeftBranchALevelFour extends SequentialCommandGroup {

    final AutonomousCommandSelector autoSelector;

    @Inject
    public FromLeftCageScoreFarLeftBranchALevelFour(AutonomousCommandSelector autoSelector,
                                                    PoseSubsystem pose, DriveSubsystem drive,
                                                    DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
                                                    MapleSimulator mapleSimulator) {
        this.autoSelector = autoSelector;

        var resetMapleSim = new InstantCommand(() -> mapleSimulator.resetPosition(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueLeftStartingLine)));

        // Force our location to start in front of left cage
        var startInFrontOfLeftCage = pose.createSetPositionCommand(
                () -> PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueLeftStartingLine)
        );
        this.addCommands(startInFrontOfLeftCage);
        this.addCommands(resetMapleSim);

        // Drive to far left, branch A and score level four
        queueMessageToAutoSelector("Drive to far left, branch A and score level four");
        var driveAndScoreFarLeftBranchALevelFour = new DeferredCommand(
                () -> driveToFaceAndScoreFact.create(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR), Set.of(drive)
        );

        this.addCommands(driveAndScoreFarLeftBranchALevelFour);
    }

    private void queueMessageToAutoSelector(String message) {
        this.addCommands(autoSelector.createAutonomousStateMessageCommand(message));
    }
}
