package competition.commandgroups;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DriveToReefFaceFromAngleUntilDetectionCommand;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import dagger.assisted.Assisted;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Provider;

import static edu.wpi.first.units.Units.Meters;

public class HeadingAssistedDriveAndScoreCommandGroup extends SequentialCommandGroup {
    CoralArmSubsystem coralArmSubsystem;


    @AssistedFactory
    public abstract static class Factory {
        public abstract HeadingAssistedDriveAndScoreCommandGroup create(@Assisted Landmarks.Branch branch);
    }

    @AssistedInject
    public HeadingAssistedDriveAndScoreCommandGroup(@Assisted Landmarks.Branch branch,
                                                    DriveToReefFaceFromAngleUntilDetectionCommand driveToReefFaceFromAngleCommand,
                                                    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                    ScoreWhenReadyCommand scoreWhenReadyCommand,
                                                    Provider<AlignToReefWithAprilTagCommand> alignToReefWithAprilTagCommandProvider,
                                                    MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand,
                                                    CoralArmSubsystem coralArmSubsystem) {
        this.coralArmSubsystem = coralArmSubsystem;
        var prepCoralSystemCommandGroup = prepCoralSystemCommandGroupFactory.create(coralArmSubsystem::getTargetCoralLevel);
        var alignToReefWithAprilTagWithCameraCommand = alignToReefWithAprilTagCommandProvider.get();

        Cameras camera = branch == Landmarks.Branch.A ? Cameras.FRONT_RIGHT_CAMERA : Cameras.FRONT_LEFT_CAMERA;

        alignToReefWithAprilTagWithCameraCommand.setConfigurations(camera.getIndex(),
                false, 0.5);
        driveToReefFaceFromAngleCommand.setAprilTagCamera(camera);
        measureDistanceBeforeScoringCommand.setBranch(branch);

        this.addCommands(driveToReefFaceFromAngleCommand);
        measureDistanceBeforeScoringCommand.setDistanceThreshold(Meters.of(1));
        var measureDistanceBeforePreppingCoralSystem = new SequentialCommandGroup(measureDistanceBeforeScoringCommand,
                prepCoralSystemCommandGroup);
        var alignWhilePrepping = new ParallelCommandGroup(alignToReefWithAprilTagWithCameraCommand,
                measureDistanceBeforePreppingCoralSystem);
        this.addCommands(alignWhilePrepping);
        this.addCommands(scoreWhenReadyCommand);
    }


}