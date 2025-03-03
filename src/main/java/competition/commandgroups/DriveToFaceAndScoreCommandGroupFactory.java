package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Provider;

import static edu.wpi.first.units.Units.Meters;

public class DriveToFaceAndScoreCommandGroupFactory {

    DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory;
    MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand;
    PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;

    DoubleProperty levelOneDistanceThreshold;
    DoubleProperty levelTwoDistanceThreshold;
    DoubleProperty levelFourDistanceThreshold;

    @Inject
    public DriveToFaceAndScoreCommandGroupFactory(DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory,
                                                  MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand,
                                                  PrepCoralSystemCommandGroupFactory prepCoralSystemFactory,
                                                  Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider,
                                                  PropertyFactory pf) {
        this.driveToReefFaceThenAlignCommandGroupFactory = driveToReefFaceThenAlignCommandGroupFactory;
        this.measureDistanceBeforeScoringCommand = measureDistanceBeforeScoringCommand;
        this.prepCoralSystemFactory = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;

        levelOneDistanceThreshold = pf.createPersistentProperty("LevelOneDistanceThresholdInMeters", 0.5);
        levelTwoDistanceThreshold = pf.createPersistentProperty("LevelTwoDistanceThresholdInMeters", 0.5);
        levelFourDistanceThreshold = pf.createPersistentProperty("LevelFourDistanceThresholdInMeters", 0.5);
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace,
                                         Landmarks.Branch targetBranch,
                                         Landmarks.CoralLevel targetLevel) {
        var driveToFaceAndScoreCommandGroup = new SequentialCommandGroup();

        var driveToFaceWhilePrepping = new ParallelCommandGroup();

        var driveToReefFaceThenAlign = driveToReefFaceThenAlignCommandGroupFactory.create(targetReefFace, targetBranch);
        var measureDistanceThenPrep = new SequentialCommandGroup();

        // TODO: Could put this inside of MeasureDistanceBeforeScoringCommand instead,
        //  but don't want to mess with John's HeadingAssistedDriveAndScoreCommandGroup right now
        double distanceThresholdInMeters = switch (targetLevel) {
            case ONE -> levelOneDistanceThreshold.get();
            case TWO -> levelTwoDistanceThreshold.get();
            default -> levelFourDistanceThreshold.get(); // For safety, the default is the shortest distance which is probably L4
        };

        measureDistanceBeforeScoringCommand.setDistanceThreshold(Meters.of(distanceThresholdInMeters));
        measureDistanceBeforeScoringCommand.setBranch(targetBranch);
        var prepCoralSystem = prepCoralSystemFactory.create(() -> targetLevel);
        measureDistanceThenPrep.addCommands(measureDistanceBeforeScoringCommand, prepCoralSystem);

        driveToFaceWhilePrepping.addCommands(driveToReefFaceThenAlign, measureDistanceThenPrep);

        var scoreWhenReady = scoreWhenReadyProvider.get();

        driveToFaceAndScoreCommandGroup.addCommands(driveToFaceWhilePrepping, scoreWhenReady);

        return driveToFaceAndScoreCommandGroup;
    }

}
