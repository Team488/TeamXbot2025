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
        // Overarching command group — drives to branch and preps coral system once the robot is close enough to the reef, scores when ready
        var driveToFaceAndScoreCommandGroup = new SequentialCommandGroup();

        // Drive to a branch while prepping the coral system once the robot is close enough
        var driveToBranchWhilePrepping = new ParallelCommandGroup();

        // Terminally approach to branch
        var driveToReefFaceThenAlign = driveToReefFaceThenAlignCommandGroupFactory.create(targetReefFace, targetBranch);

        // Prep coral system once robot is within a distance threshold
        var measureDistanceThenPrep = new SequentialCommandGroup();
        double distanceThresholdInMeters = switch (targetLevel) { // Distance threshold may be much bigger for lower levels
            case ONE -> levelOneDistanceThreshold.get();
            case TWO -> levelTwoDistanceThreshold.get();
            default -> levelFourDistanceThreshold.get(); // For safety, the default is the shortest distance which is probably L4
        };
        measureDistanceBeforeScoringCommand.setDistanceThreshold(Meters.of(distanceThresholdInMeters));
        measureDistanceBeforeScoringCommand.setBranch(targetBranch);
        var prepCoralSystem = prepCoralSystemFactory.create(() -> targetLevel);
        measureDistanceThenPrep.addCommands(measureDistanceBeforeScoringCommand, prepCoralSystem);

        driveToBranchWhilePrepping.addCommands(driveToReefFaceThenAlign, measureDistanceThenPrep);

        var scoreWhenReady = scoreWhenReadyProvider.get();

        driveToFaceAndScoreCommandGroup.addCommands(driveToBranchWhilePrepping, scoreWhenReady);

        return driveToFaceAndScoreCommandGroup;
    }

}
