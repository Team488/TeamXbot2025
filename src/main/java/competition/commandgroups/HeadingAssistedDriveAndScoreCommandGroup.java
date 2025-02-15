package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DriveToReefFaceFromAngleUntilDetectionCommand;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class HeadingAssistedDriveAndScoreCommandGroup extends SequentialCommandGroup {
    PoseSubsystem pose;
    Landmarks.CoralLevel targetCoralLevel = Landmarks.CoralLevel.COLLECTING;

    @Inject
    public HeadingAssistedDriveAndScoreCommandGroup(DriveToReefFaceFromAngleUntilDetectionCommand driveToReefFaceFromAngleCommand,
                                                    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                    ScoreWhenReadyCommand scoreWhenReadyCommand,
                                                    AlignToReefWithAprilTagCommand alignToReefWithAprilTagCommand,
                                                    MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand,
                                                    PoseSubsystem pose) {
        this.pose = pose;
        this.addCommands(driveToReefFaceFromAngleCommand);
        var prep = prepCoralSystemCommandGroupFactory.create(pose::getTargetCoralLevel);

//        var measureDistanceBeforePrep = new SequentialCommandGroup(measureDistanceBeforeScoringCommand, prep);
//        var alignWhilePrepping = new ParallelCommandGroup(alignToReefWithAprilTagCommand, measureDistanceBeforePrep);
//        this.addCommands(alignWhilePrepping);
        this.addCommands(prep);
//        this.addCommands(scoreWhenReadyCommand);
    }
}
