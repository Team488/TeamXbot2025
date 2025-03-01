package competition.commandgroups;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class DriveToReefAndScoreCommandGroupFactory {
    DriveToReefThenAlignCommandGroupFactory driveToReefThenAlignCommandGroupFactory;
    PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;

    @Inject
    public DriveToReefAndScoreCommandGroupFactory(DriveToReefThenAlignCommandGroupFactory driveToReefThenAlignCommandGroupFactory,
                                                  PrepCoralSystemCommandGroupFactory prepCoralSystemFactory,
                                                  Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider) {
        this.driveToReefThenAlignCommandGroupFactory = driveToReefThenAlignCommandGroupFactory;
        this.prepCoralSystemFactory = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;
    }

    public SequentialCommandGroup create(Landmarks.Branch targetBranch,
                                         Landmarks.CoralLevel targetLevel) {
        var driveToFaceAndScoreCommandGroup = new SequentialCommandGroup();

        var driveToReefWhilePrepping = new ParallelCommandGroup();

        var driveToReefFaceThenAlign = driveToReefThenAlignCommandGroupFactory.create(targetBranch);
        var prepCoralSystem = prepCoralSystemFactory.create(() -> targetLevel);

        driveToReefWhilePrepping.addCommands(driveToReefFaceThenAlign, prepCoralSystem);

        var scoreWhenReady = scoreWhenReadyProvider.get();

        driveToFaceAndScoreCommandGroup.addCommands(driveToReefWhilePrepping, scoreWhenReady);

        return driveToFaceAndScoreCommandGroup;
    }
}
