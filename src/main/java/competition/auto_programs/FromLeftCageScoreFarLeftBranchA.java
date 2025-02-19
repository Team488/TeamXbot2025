package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;

public class FromLeftCageScoreFarLeftBranchA extends SequentialCommandGroup {

    final AutonomousCommandSelector autoSelector;

    @Inject
    public FromLeftCageScoreFarLeftBranchA(AutonomousCommandSelector autoSelector,
                                           PoseSubsystem pose,
                                           DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact) {
        this.autoSelector = autoSelector;

        // Force our location to start in front of left cage
        var startInFrontOfLeftCage = pose.createSetPositionCommand(
                () -> Landmarks.BlueLeftStartingLine
        );
        this.addCommands(startInFrontOfLeftCage);

        // Drive to far left, branch A and score level four
        queueMessageToAutoSelector("Drive to far left, branch A and score level four");
        var driveAndScoreFarLeftBranchALevelFour = driveToFaceAndScoreFact.create(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreFarLeftBranchALevelFour);
    }

    private void queueMessageToAutoSelector(String message) {
        this.addCommands(autoSelector.createAutonomousStateMessageCommand(message));
    }
}
