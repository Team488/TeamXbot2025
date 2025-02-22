package competition.commandgroups;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

import static edu.wpi.first.units.Units.Meters;

public class DriveToFaceAndScoreCommandGroupFactory {

    DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory;
    PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;
    MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand;

    @Inject
    public DriveToFaceAndScoreCommandGroupFactory(DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory,
                                                  PrepCoralSystemCommandGroupFactory prepCoralSystemFactory,
                                                  Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider,
                                                  MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand) {
        this.driveToReefFaceThenAlignCommandGroupFactory = driveToReefFaceThenAlignCommandGroupFactory;
        this.prepCoralSystemFactory = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;
        this.measureDistanceBeforeScoringCommand = measureDistanceBeforeScoringCommand;
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace,
                                         Landmarks.Branch targetBranch,
                                         Landmarks.CoralLevel targetLevel) {
        var driveToFaceAndScoreCommandGroup = new SequentialCommandGroup();

        var driveToReefWhilePrepping = new ParallelCommandGroup();

        var driveToReefFaceThenAlign = driveToReefFaceThenAlignCommandGroupFactory.create(targetReefFace, targetBranch);
        var prepCoralSystem = prepCoralSystemFactory.create(() -> targetLevel);
        measureDistanceBeforeScoringCommand.setDistanceThreshold(Meters.of(1));
        var measureDistanceBeforePreppingSystem = new SequentialCommandGroup(measureDistanceBeforeScoringCommand, prepCoralSystem,
                scoreWhenReadyProvider.get());

        driveToReefWhilePrepping.addCommands(driveToReefFaceThenAlign, measureDistanceBeforePreppingSystem);

        driveToFaceAndScoreCommandGroup.addCommands(driveToReefWhilePrepping);

        return driveToFaceAndScoreCommandGroup;
    }

}
