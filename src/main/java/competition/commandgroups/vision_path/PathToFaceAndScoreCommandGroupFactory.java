package competition.commandgroups.vision_path;

import static edu.wpi.first.units.Units.Meters;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import javax.inject.Inject;
import javax.inject.Provider;

public class PathToFaceAndScoreCommandGroupFactory {
    PathToReefFaceThenAlignCommandGroupFactory
            driveToReefFaceThenAlignCommandGroupFactory;
    PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;
    Provider<MeasureDistanceBeforeScoringCommand> measureDistanceBeforeScoringCommandProvider;

    @Inject
    public PathToFaceAndScoreCommandGroupFactory(
            PathToReefFaceThenAlignCommandGroupFactory
                    driveToReefFaceThenAlignCommandGroupFactory,
            PrepCoralSystemCommandGroupFactory prepCoralSystemFactory,
            Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider,
            Provider<MeasureDistanceBeforeScoringCommand> measureDistanceBeforeScoringCommandProvider) {
        this.driveToReefFaceThenAlignCommandGroupFactory =
                driveToReefFaceThenAlignCommandGroupFactory;
        this.prepCoralSystemFactory = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;
        this.measureDistanceBeforeScoringCommandProvider =
        measureDistanceBeforeScoringCommandProvider;
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace,
                                         Landmarks.Branch targetBranch, Landmarks.CoralLevel targetLevel) {
        var driveToFaceAndScoreCommandGroup = new SequentialCommandGroup();

        var driveToReefWhilePrepping = new ParallelCommandGroup();

        var driveToReefFaceThenAlign =
                driveToReefFaceThenAlignCommandGroupFactory.create(
                        targetReefFace, targetBranch);
        var prepCoralSystem = prepCoralSystemFactory.create(() -> targetLevel);
        var measureDistanceBeforeScoringCommand = measureDistanceBeforeScoringCommandProvider.get();
        measureDistanceBeforeScoringCommand.setDistanceThreshold(Meters.of(1));
        var measureDistanceBeforePreppingSystem =
                new SequentialCommandGroup(measureDistanceBeforeScoringCommand,
                        prepCoralSystem, scoreWhenReadyProvider.get());

        driveToReefWhilePrepping.addCommands(
                driveToReefFaceThenAlign, measureDistanceBeforePreppingSystem);

        driveToFaceAndScoreCommandGroup.addCommands(driveToReefWhilePrepping);

        return driveToFaceAndScoreCommandGroup;
    }
}
