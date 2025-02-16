package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DriveToReefFaceFromAngleUntilDetectionCommand;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class HeadingAssistedDriveAndScoreCommandGroup extends SequentialCommandGroup {
    PoseSubsystem pose;
    Landmarks.Branch branch;

    @Inject
    public HeadingAssistedDriveAndScoreCommandGroup(DriveToReefFaceFromAngleUntilDetectionCommand driveToReefFaceFromAngleCommand,
                                                    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                    ScoreWhenReadyCommand scoreWhenReadyCommand,
                                                    Provider<AlignToReefWithAprilTagCommand> alignToReefWithAprilTagCommandProvider,
                                                    MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand,
                                                    PoseSubsystem pose) {
        this.pose = pose;
        this.addCommands(driveToReefFaceFromAngleCommand);
        var prep = prepCoralSystemCommandGroupFactory.create(pose::getTargetCoralLevel);
        var alignToReefWithAprilTagWithCamera = alignToReefWithAprilTagCommandProvider.get();

        if (branch == Landmarks.Branch.A) {
            alignToReefWithAprilTagWithCamera.setConfigurations(1, false, 0.5);
        }
        else {
            alignToReefWithAprilTagWithCamera.setConfigurations(0, false, 0.5);
        }

        var measureDistanceBeforePrep = new SequentialCommandGroup(measureDistanceBeforeScoringCommand, prep);
        var alignWhilePrepping = new ParallelCommandGroup(alignToReefWithAprilTagWithCamera, measureDistanceBeforePrep);
        this.addCommands(alignWhilePrepping);
        this.addCommands(scoreWhenReadyCommand);
    }

    public void setBranch(Landmarks.Branch branch) {
        this.branch = branch;
    }
}
