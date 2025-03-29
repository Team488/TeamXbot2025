package competition.commandgroups.vision_path;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.vision_path.PathDriveToReefFaceCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToReefFaceUntilAprilTagDetection;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.kobe.xbot.Utilities.Entities.XTableValues;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.Set;

public class PathToReefFaceAndPrepThenAlignCommandGroupFactory {
    Provider<PathDriveToReefFaceUntilAprilTagDetection> driveToReefFaceCommandProvider;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    Provider<AlignToTagGlobalMovementWithCalculator> alignToReefWithAprilTagCommandProvider;
    DriveSubsystem drive;

    @Inject
    public PathToReefFaceAndPrepThenAlignCommandGroupFactory(
            Provider<AlignToTagGlobalMovementWithCalculator> alignToReefWithAprilTagCommandProvider,
            Provider<PathDriveToReefFaceUntilAprilTagDetection> driveToReefFaceCommandProvider,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
            DriveSubsystem drive) {
        this.driveToReefFaceCommandProvider = driveToReefFaceCommandProvider;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.alignToReefWithAprilTagCommandProvider = alignToReefWithAprilTagCommandProvider;
        this.drive = drive;
    }

    public void setBranch(AlignToTagGlobalMovementWithCalculator command, Landmarks.ReefFace reefFace, Landmarks.Branch branch) {
        if (branch == Landmarks.Branch.A) {
            command.setConfigurations(Cameras.FRONT_RIGHT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, 1,
                    AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, false, true);
        } else {
            command.setConfigurations(Cameras.FRONT_LEFT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, 1,
                    AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, false, true);
        }
    }

    public SequentialCommandGroup create(
            Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch) {
        return create(targetReefFace, targetBranch, null);
    }

    public SequentialCommandGroup create(
            Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch, XTableValues.BezierCurves overridden) {
        PathDriveToReefFaceCommand driveToReefFaceCommand = driveToReefFaceCommandProvider.get();
        if (overridden != null) {
            driveToReefFaceCommand.setOverriddenPath(overridden);
        }
        var alignToReefCommand = new DeferredCommand(
                () -> {
                    var alignToReefWithAprilTagCommand = alignToReefWithAprilTagCommandProvider.get();
                    setBranch(alignToReefWithAprilTagCommand, targetReefFace, targetBranch);
                    return alignToReefWithAprilTagCommand;
                }, Set.of(drive)
        );
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveToReefFaceCommand.setReefFace(targetReefFace)
                        .setBranch(targetBranch)),
                driveToReefFaceCommand,
                alignToReefCommand);
    }

}
