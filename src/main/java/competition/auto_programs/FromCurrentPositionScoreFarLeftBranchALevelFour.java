package competition.auto_programs;

import competition.commandgroups.PathToFaceAndScoreCommandGroupFactory;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;

public class FromCurrentPositionScoreFarLeftBranchALevelFour extends SequentialCommandGroup {

    final AutonomousCommandSelector autoSelector;

    @Inject
    public FromCurrentPositionScoreFarLeftBranchALevelFour(AutonomousCommandSelector autoSelector,
                                                    PoseSubsystem pose,
                                                           PathToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact) {
        this.autoSelector = autoSelector;

//        var resetMapleSimPose = new InstantCommand(() -> mapleSimulator.resetPosition(Landmarks.BlueLeftStartingLine));
//        this.addCommands(resetMapleSimPose);

        // Force our location to start in front of left cage
//        var startInFrontOfLeftCage = pose.createSetPositionCommand(
//                () -> Landmarks.BlueLeftStartingLine
//        );
//        this.addCommands(startInFrontOfLeftCage);

        // Drive to far left, branch A and score level four
        queueMessageToAutoSelector("Path curve to far left, branch A and score level four");
        var pathAndCurveScoreFarLeftBranchALevelFour = driveToFaceAndScoreFact.create(Landmarks.ReefFace.FAR_LEFT,
                Landmarks.Branch.A, Landmarks.CoralLevel.TWO);
        this.addCommands(pathAndCurveScoreFarLeftBranchALevelFour);
    }

    private void queueMessageToAutoSelector(String message) {
        this.addCommands(autoSelector.createAutonomousStateMessageCommand(message));
    }
}
