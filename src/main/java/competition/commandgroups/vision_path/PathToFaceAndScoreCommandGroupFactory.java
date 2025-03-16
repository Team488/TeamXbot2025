package competition.commandgroups.vision_path;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

import static edu.wpi.first.units.Units.Meters;

public class PathToFaceAndScoreCommandGroupFactory {
    Provider<PathToReefFaceThenAlignCommandGroupFactory>
            pathDriveToReefFaceThenAlignCommandGroupFactoryProvider;
    Provider<PrepCoralSystemCommandGroupFactory> prepCoralSystemCommandGroupFactoryProvider;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;
    Provider<MeasureDistanceBeforeScoringCommand> measureDistanceBeforeScoringCommandProvider;

    @Inject
    public PathToFaceAndScoreCommandGroupFactory(
            Provider<PathToReefFaceThenAlignCommandGroupFactory>
                    driveToReefFaceThenAlignCommandGroupFactoryProvider,
            Provider<PrepCoralSystemCommandGroupFactory> prepCoralSystemFactory,
            Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider,
            Provider<MeasureDistanceBeforeScoringCommand> measureDistanceBeforeScoringCommandProvider) {
        this.pathDriveToReefFaceThenAlignCommandGroupFactoryProvider =
                driveToReefFaceThenAlignCommandGroupFactoryProvider;
        this.prepCoralSystemCommandGroupFactoryProvider = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;
        this.measureDistanceBeforeScoringCommandProvider =
        measureDistanceBeforeScoringCommandProvider;
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace,
                                         Landmarks.Branch targetBranch, Landmarks.CoralLevel targetLevel) {
        var driveToFaceAndScoreCommandGroup = new SequentialCommandGroup();
        driveToFaceAndScoreCommandGroup.setName("DriveToFaceAndScore");
        var driveToReefWhilePrepping = new ParallelCommandGroup();
        driveToReefWhilePrepping.setName("DriveToBranchWhilePrepping");

        PathToReefFaceThenAlignCommandGroupFactory pathToReefFaceThenAlignCommandGroupFactory = pathDriveToReefFaceThenAlignCommandGroupFactoryProvider.get();
        PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactoryProvider.get();

        var driveToReefFaceThenAlign =
                pathToReefFaceThenAlignCommandGroupFactory.create(
                        targetReefFace, targetBranch);
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> targetLevel);

        var raiseThrou =
                new SequentialCommandGroup(
                        prepCoralSystem, scoreWhenReadyProvider.get());
        raiseThrou.setName("RaiseThrou");

        driveToReefWhilePrepping.addCommands(
                driveToReefFaceThenAlign);

        driveToFaceAndScoreCommandGroup.addCommands(driveToReefWhilePrepping);

        return driveToFaceAndScoreCommandGroup;
    }
}
