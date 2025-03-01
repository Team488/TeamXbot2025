package competition.commandgroups;

import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.DriveToReefFaceUntilDetectionCommand;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.controls.sensors.XXboxController;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.Set;

public class DriveToReefThenAlignCommandGroupFactory {

    DriveToReefFaceUntilDetectionCommand driveToReefFaceCommand;
    AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;

    @Inject
    public DriveToReefThenAlignCommandGroupFactory(DriveToReefFaceUntilDetectionCommand driveToReefFaceCommand,
            AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        this.driveToReefFaceCommand = driveToReefFaceCommand;
        this.alignToReefWithAprilTagCommand = alignToReefWithAprilTagCommand;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public void setBranch(Landmarks.Branch branch) {
        // TODO: Remove need for reefface
        if (branch == Landmarks.Branch.A) {
            alignToReefWithAprilTagCommand.setConfigurations(Cameras.FRONT_RIGHT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(Landmarks.ReefFace.CLOSE), false, 1);
        } else {
            alignToReefWithAprilTagCommand.setConfigurations(Cameras.FRONT_LEFT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(Landmarks.ReefFace.CLOSE), false, 1);
        }
    }

    public SequentialCommandGroup create(Landmarks.Branch targetBranch) {
        var group = new SequentialCommandGroup();

        // TODO: Remove need for reefface
        driveToReefFaceCommand.setTargetReefFacePose(Landmarks.ReefFace.CLOSE);
        var alignToReefCommand = new DeferredCommand(
                () -> {
                    setBranch(targetBranch);
                    return alignToReefWithAprilTagCommand;
                }, Set.of());
        group.addCommands(driveToReefFaceCommand, alignToReefCommand);

        return group;
    }
}
