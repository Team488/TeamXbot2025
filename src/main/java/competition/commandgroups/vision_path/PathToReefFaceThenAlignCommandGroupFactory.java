package competition.commandgroups.vision_path;

import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.vision_path.PathDriveToLocationCommandUntilAprilTagDetection;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class PathToReefFaceThenAlignCommandGroupFactory {
    Provider<PathDriveToLocationCommandUntilAprilTagDetection> driveToReefFaceCommandProvider;
    Provider<AlignToTagGlobalMovementWithCalculator> alignToReefWithAprilTagCommandProvider;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;


    @Inject
    public PathToReefFaceThenAlignCommandGroupFactory(
            Provider<PathDriveToLocationCommandUntilAprilTagDetection> driveToReefFaceCommandProvider,
            Provider<AlignToTagGlobalMovementWithCalculator> alignToReefWithAprilTagCommandProvider,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        this.driveToReefFaceCommandProvider = driveToReefFaceCommandProvider;
        this.alignToReefWithAprilTagCommandProvider = alignToReefWithAprilTagCommandProvider;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public void setBranch(Landmarks.ReefFace reefFace, Landmarks.Branch branch,
                          AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand) {
        if (branch == Landmarks.Branch.A) {
            alignToReefWithAprilTagCommand.setConfigurations(
                    Cameras.FRONT_RIGHT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, -2);
        } else {
            alignToReefWithAprilTagCommand.setConfigurations(
                    Cameras.FRONT_LEFT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, -2);
        }
    }

    public SequentialCommandGroup create(
            Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch) {
        var group = new SequentialCommandGroup();
        PathDriveToLocationCommandUntilAprilTagDetection driveToReefFaceCommand = driveToReefFaceCommandProvider.get();
        AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand = alignToReefWithAprilTagCommandProvider.get();
        driveToReefFaceCommand.setTarget(targetReefFace, targetBranch,
                aprilTagVisionSubsystem.getTargetAprilTagID(targetReefFace));
        setBranch(targetReefFace, targetBranch, alignToReefWithAprilTagCommand);
        group.addCommands(driveToReefFaceCommand, alignToReefWithAprilTagCommand);

        return group;
    }

}
