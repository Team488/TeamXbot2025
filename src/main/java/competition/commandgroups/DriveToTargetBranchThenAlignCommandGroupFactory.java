package competition.commandgroups;

import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.DriveToTargetBranchUntilDetectionCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToLocationCommandUntilAprilTagDetection;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.command.BaseCommand;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.Set;

public class DriveToTargetBranchThenAlignCommandGroupFactory {

    Provider<PathDriveToLocationCommandUntilAprilTagDetection> pathDriveToLocationCommandUntilAprilTagDetectionProv;
    Provider<AlignToTagGlobalMovementWithCalculator> alignToTagGlobalMovementWithCalculatorProv;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    PoseSubsystem pose;

    Provider<DriveToTargetBranchUntilDetectionCommand> driveToTargetBranchUntilDetectionCommandProv;

    @Inject
    public DriveToTargetBranchThenAlignCommandGroupFactory(Provider<PathDriveToLocationCommandUntilAprilTagDetection> pathDriveToLocationCommandUntilAprilTagDetectionProv,
                                                           Provider<AlignToTagGlobalMovementWithCalculator> alignToTagGlobalMovementWithCalculatorProv,
                                                           AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
                                                           PoseSubsystem pose,
                                                           Provider<DriveToTargetBranchUntilDetectionCommand> driveToTargetBranchUntilDetectionCommandProv) {
        this.pathDriveToLocationCommandUntilAprilTagDetectionProv = pathDriveToLocationCommandUntilAprilTagDetectionProv;
        this.alignToTagGlobalMovementWithCalculatorProv = alignToTagGlobalMovementWithCalculatorProv;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.pose = pose;

        this.driveToTargetBranchUntilDetectionCommandProv = driveToTargetBranchUntilDetectionCommandProv;
    }

    public void setBranch(AlignToTagGlobalMovementWithCalculator command, Landmarks.ReefFace reefFace, Landmarks.Branch branch) {
        if (branch == Landmarks.Branch.A) {
            command.setConfigurations(Cameras.FRONT_RIGHT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, 1,
                    AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, false);
        }
        else {
            command.setConfigurations(Cameras.FRONT_LEFT_CAMERA.getIndex(),
                    aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, 1,
                    AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, false);
        }
    }

    public SequentialCommandGroup create() {
        var driveAndAlignToTargetBranch = new SequentialCommandGroup();

        var driveToTargetBranchUntilDetection = new DeferredCommand(
                () -> {
                    var driveUntilDetection = pathDriveToLocationCommandUntilAprilTagDetectionProv.get();
                    driveUntilDetection.setTarget(pose.getTargetReefFaceAndBranch().get().reefFace(),
                            pose.getTargetReefFaceAndBranch().get().branch(),
                            aprilTagVisionSubsystem.getTargetAprilTagID(pose.getTargetReefFaceAndBranch().get().reefFace()));
                    return driveUntilDetection;
//                    var driveUntilDetection = pathDriveToLocationCommandUntilAprilTagDetectionProv.get();
//                    driveUntilDetection.setTarget(Landmarks.ReefFace.CLOSE,
//                            Landmarks.Branch.A,
//                            aprilTagVisionSubsystem.getTargetAprilTagID(Landmarks.ReefFace.CLOSE));
//                    return driveUntilDetection;
                }, Set.of()
        );

        var driveToTargetBranchUntilDetectionNoBezier = new DeferredCommand(
                () -> {
                    var driveToTargetBranchUntilDetectionCommand = driveToTargetBranchUntilDetectionCommandProv.get();
                    driveToTargetBranchUntilDetectionCommand.setAprilTagCamera(pose.getTargetReefFaceAndBranch().get().branch() == Landmarks.Branch.A ? Cameras.FRONT_RIGHT_CAMERA
                            : Cameras.FRONT_LEFT_CAMERA);
                    driveToTargetBranchUntilDetectionCommand.setTargetReefFacePose(Landmarks.getReefFacePose(pose.getTargetReefFaceAndBranch().get().reefFace()));

                    return driveToTargetBranchUntilDetectionCommand;
                }, Set.of()
        );

        var alignToTagGlobalMovementWithCalculator = new DeferredCommand(
                () -> {
                    var alignToReefWithAprilTagCommand = alignToTagGlobalMovementWithCalculatorProv.get();
                    setBranch(alignToReefWithAprilTagCommand, pose.getTargetReefFaceAndBranch().get().reefFace(),
                            pose.getTargetReefFaceAndBranch().get().branch());
                    return alignToReefWithAprilTagCommand;
//                    var alignToReefWithAprilTagCommand = alignToTagGlobalMovementWithCalculatorProv.get();
//                    setBranch(alignToReefWithAprilTagCommand, Landmarks.ReefFace.CLOSE, Landmarks.Branch.A);
//                    return alignToReefWithAprilTagCommand;
                }, Set.of() // TODO: Do I need to add the DriveSubsystem as a requirement?
        );

        driveAndAlignToTargetBranch.addCommands(driveToTargetBranchUntilDetection, alignToTagGlobalMovementWithCalculator);

        return driveAndAlignToTargetBranch;
    }
}
