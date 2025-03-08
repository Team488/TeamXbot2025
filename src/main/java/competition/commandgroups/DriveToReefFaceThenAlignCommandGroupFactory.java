package competition.commandgroups;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.DriveHermiteSplineCommand;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.Set;

public class DriveToReefFaceThenAlignCommandGroupFactory {

    final Provider<AlignToTagGlobalMovementWithCalculator> alignToReefWithAprilTagCommandProvider;
    final Provider<DriveHermiteSplineCommand> splineDriveFactory;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    DriveSubsystem drive;

    @Inject
    public DriveToReefFaceThenAlignCommandGroupFactory(Provider<AlignToTagGlobalMovementWithCalculator> alignToReefWithAprilTagCommandProvider,
                                                       Provider<DriveHermiteSplineCommand> splineDriveFactory,
                                                       AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
                                                       DriveSubsystem drive) {
        this.alignToReefWithAprilTagCommandProvider = alignToReefWithAprilTagCommandProvider;
        this.splineDriveFactory = splineDriveFactory;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.drive = drive;
    }

    public void setBranch(AlignToTagGlobalMovementWithCalculator command, Landmarks.ReefFace reefFace, Landmarks.Branch branch) {
        int cameraIndex = 0;
        if (branch == Landmarks.Branch.A) {
            cameraIndex = Cameras.FRONT_RIGHT_CAMERA.getIndex();
        }
        else {
            cameraIndex = Cameras.FRONT_LEFT_CAMERA.getIndex();
        }

        command.setConfigurations(cameraIndex,
                () -> aprilTagVisionSubsystem.getTargetAprilTagID(reefFace), false, 1,
                AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering, false);
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch) {
        return create(targetReefFace, targetBranch, false);
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch, boolean useSplinesForRouting) {
        var group = new SequentialCommandGroup();
        if (useSplinesForRouting) {
            var splineDriveCommand = splineDriveFactory.get();
            splineDriveCommand.configureForReef(targetReefFace, targetBranch);
            group.addCommands(splineDriveCommand);
        }

        var alignToReefWithAprilTagCommand = alignToReefWithAprilTagCommandProvider.get();
        setBranch(alignToReefWithAprilTagCommand, targetReefFace, targetBranch);

        group.addCommands(alignToReefWithAprilTagCommand);
        return group;
    }
}
