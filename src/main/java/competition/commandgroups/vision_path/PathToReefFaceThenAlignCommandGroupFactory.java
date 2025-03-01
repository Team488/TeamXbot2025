package competition.commandgroups.vision_path;

import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.vision_path.PathDriveToLocationUntilAprilTagDetection;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import javax.inject.Inject;

public class PathToReefFaceThenAlignCommandGroupFactory {
    PathDriveToLocationUntilAprilTagDetection driveToReefFaceCommand;
    AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;

    @Inject
    public PathToReefFaceThenAlignCommandGroupFactory(
            PathDriveToLocationUntilAprilTagDetection driveToReefFaceCommand,
            AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        this.driveToReefFaceCommand = driveToReefFaceCommand;
        this.alignToReefWithAprilTagCommand = alignToReefWithAprilTagCommand;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public void setBranch(Landmarks.ReefFace reefFace, Landmarks.Branch branch) {
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

        driveToReefFaceCommand.setTarget(targetReefFace.getViewable(),
                aprilTagVisionSubsystem.getTargetAprilTagID(targetReefFace));
        setBranch(targetReefFace, targetBranch);
        group.addCommands(driveToReefFaceCommand, alignToReefWithAprilTagCommand);

        return group;
    }
}
