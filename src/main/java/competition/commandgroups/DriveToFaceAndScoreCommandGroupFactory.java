package competition.commandgroups;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class DriveToFaceAndScoreCommandGroupFactory {

    DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory;
     PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;

    @Inject
    public DriveToFaceAndScoreCommandGroupFactory(DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory,
                                                  PrepCoralSystemCommandGroupFactory prepCoralSystemFactory,
                                                  Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider) {
        this.driveToReefFaceThenAlignCommandGroupFactory = driveToReefFaceThenAlignCommandGroupFactory;
        this.prepCoralSystemFactory = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace,
                                         Landmarks.Branch targetBranch,
                                         ElevatorSubsystem.ElevatorGoals elevatorGoal,
                                         CoralArmPivotSubsystem.ArmGoals armGoal) {
        var driveToFaceAndScoreCommandGroup = new SequentialCommandGroup();

        var driveToReefWhilePrepping = new ParallelCommandGroup();
        var driveToReefFaceThenAlign = driveToReefFaceThenAlignCommandGroupFactory.create(targetReefFace, targetBranch);
        var prepCoralSystem = prepCoralSystemFactory.create(elevatorGoal, armGoal);
        driveToReefWhilePrepping.addCommands(driveToReefFaceThenAlign, prepCoralSystem);

        var scoreWhenReady = scoreWhenReadyProvider.get();

        driveToFaceAndScoreCommandGroup.addCommands(driveToReefWhilePrepping, scoreWhenReady);

        return driveToFaceAndScoreCommandGroup;
    }

}
