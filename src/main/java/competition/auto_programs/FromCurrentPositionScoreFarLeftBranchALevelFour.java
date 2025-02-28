package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.DriveToStationAndIntakeUntilCollectedCommandGroupFactory;
import competition.commandgroups.PathToFaceAndScoreCommandGroupFactory;
import competition.simulation.MapleSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;

public class FromCurrentPositionScoreFarLeftBranchALevelFour extends SequentialCommandGroup {

    final AutonomousCommandSelector autoSelector;

    @Inject
    public FromCurrentPositionScoreFarLeftBranchALevelFour(AutonomousCommandSelector autoSelector,
                                                           PoseSubsystem pose,
                                                           MapleSimulator mapleSimulator,
                                                           PathToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
                                                           DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreCommandGroupFactory,
                                                           DriveToStationAndIntakeUntilCollectedCommandGroupFactory driveToStationAndIntakeUntilCollectedCommandGroupFactory) {
        this.autoSelector = autoSelector;

//        var resetMapleSimPose = new InstantCommand(() -> mapleSimulator.resetPosition(Landmarks.BlueLeftStartingLine));
//        this.addCommands(resetMapleSimPose);
//
//        var startInFrontOfLeftCage = pose.createSetPositionCommand(
//                () -> Landmarks.BlueLeftStartingLine
//        );
//        this.addCommands(startInFrontOfLeftCage);

        // Drive to far left, branch A and score level four
        queueMessageToAutoSelector("Path curve to far left, branch A and score level four");
        var pathAndCurveScoreFarLeftBranchALevelFour = driveToFaceAndScoreFact.create(Landmarks.ReefFace.FAR_LEFT,
                Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        this.addCommands(pathAndCurveScoreFarLeftBranchALevelFour);
        this.addCommands(driveToStationAndIntakeUntilCollectedCommandGroupFactory.create(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        var driveAndScoreCloseLeftBranchALevelFour = driveToFaceAndScoreCommandGroupFactory.create(Landmarks.ReefFace.CLOSE_LEFT,
               Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);
    }

    private void queueMessageToAutoSelector(String message) {
        this.addCommands(autoSelector.createAutonomousStateMessageCommand(message));
    }
}
