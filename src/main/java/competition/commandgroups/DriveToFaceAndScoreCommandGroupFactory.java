package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.properties.DistanceProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Provider;

import static edu.wpi.first.units.Units.Meters;

public class DriveToFaceAndScoreCommandGroupFactory {

    DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory;
    Provider<MeasureDistanceBeforeScoringCommand> measureDistanceBeforeScoringCommandProvider;
    PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;

    DistanceProperty levelOneDistanceThreshold;
    DistanceProperty levelTwoDistanceThreshold;
    DistanceProperty levelFourDistanceThreshold;

    @Inject
    public DriveToFaceAndScoreCommandGroupFactory(DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory,
                                                  Provider<MeasureDistanceBeforeScoringCommand> measureDistanceBeforeScoringCommandProvider,
                                                  PrepCoralSystemCommandGroupFactory prepCoralSystemFactory,
                                                  Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider,
                                                  PropertyFactory pf) {
        this.driveToReefFaceThenAlignCommandGroupFactory = driveToReefFaceThenAlignCommandGroupFactory;
        this.measureDistanceBeforeScoringCommandProvider = measureDistanceBeforeScoringCommandProvider;
        this.prepCoralSystemFactory = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;

        pf.setPrefix("DriveToFaceAndScoreCommandGroupFactory");
        levelOneDistanceThreshold = pf.createPersistentProperty("LevelOneDistanceThresholdInMeters", Meters.of(2));
        levelTwoDistanceThreshold = pf.createPersistentProperty("LevelTwoDistanceThresholdInMeters", Meters.of(2));
        levelFourDistanceThreshold = pf.createPersistentProperty("LevelFourDistanceThresholdInMeters",  Meters.of(1.5));

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
        Distance distanceThresholdInMeters = switch (targetLevel) { // Distance threshold may be much bigger for lower levels
            case ONE -> levelOneDistanceThreshold.get();
            case TWO -> levelTwoDistanceThreshold.get();
            default -> levelFourDistanceThreshold.get(); // For safety, the default is the shortest distance which is probably L4
        };

        var measureDistanceBeforeScoringCommand = measureDistanceBeforeScoringCommandProvider.get();

        measureDistanceBeforeScoringCommand.setDistanceThreshold(distanceThresholdInMeters);
        measureDistanceBeforeScoringCommand.setBranch(targetBranch);
        var prepCoralSystem = prepCoralSystemFactory.create(() -> targetLevel);
        measureDistanceThenPrep.addCommands(measureDistanceBeforeScoringCommand, prepCoralSystem);

        driveToBranchWhilePrepping.addCommands(driveToReefFaceThenAlign, measureDistanceThenPrep);

        var scoreWhenReady = scoreWhenReadyProvider.get();
        var waitBeforeScoring = new WaitCommand(1); // Wait for the wobble to go away

        driveToFaceAndScoreCommandGroup.addCommands(driveToBranchWhilePrepping, waitBeforeScoring, scoreWhenReady);

        return driveToFaceAndScoreCommandGroup;
    }

}
