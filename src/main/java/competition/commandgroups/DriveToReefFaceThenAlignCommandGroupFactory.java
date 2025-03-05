package competition.commandgroups;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.DriveToReefFaceUntilDetectionCommand;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.controls.sensors.XXboxController;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.Set;

public class DriveToReefFaceThenAlignCommandGroupFactory {

    DriveToReefFaceUntilDetectionCommand driveToReefFaceCommand;
    AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    DriveSubsystem drive;

    @Inject
    public DriveToReefFaceThenAlignCommandGroupFactory(DriveToReefFaceUntilDetectionCommand driveToReefFaceCommand,
                                                       AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand,
                                                       AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
                                                       DriveSubsystem drive) {
        this.driveToReefFaceCommand = driveToReefFaceCommand;
        this.alignToReefWithAprilTagCommand = alignToReefWithAprilTagCommand;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.drive = drive;
    }

    public void setBranch(Landmarks.ReefFace reefFace, Landmarks.Branch branch) {
        if (branch == Landmarks.Branch.A) {
            alignToReefWithAprilTagCommand.setConfigurations(Cameras.FRONT_RIGHT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, 1,
                    AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, true);
        }
        else {
            alignToReefWithAprilTagCommand.setConfigurations(Cameras.FRONT_LEFT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, 1,
                    AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, true);
        }
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch) {
        var group = new SequentialCommandGroup();

        driveToReefFaceCommand.setTargetReefFacePose(targetReefFace);
        var alignToReefCommand = new DeferredCommand(
                () -> {
                    setBranch(targetReefFace, targetBranch);
                    return alignToReefWithAprilTagCommand;
                }, Set.of(drive)
        );
        group.addCommands(
                alignToReefCommand);

        return group;
    }
}
