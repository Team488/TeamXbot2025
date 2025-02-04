package competition.commandgroups;

import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DriveToReefFaceUntilDetectionCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class DriveToReefFaceThenAlignCommandGroupFactory {

    Provider<DriveToReefFaceUntilDetectionCommand> driveToReefFaceCommandProvider;
    Provider<AlignToReefWithAprilTagCommand> alignToReefWithAprilTagCommandProvider;

    @Inject
    public DriveToReefFaceThenAlignCommandGroupFactory(Provider<DriveToReefFaceUntilDetectionCommand> driveToReefFaceCommandProvider,
                                                       Provider<AlignToReefWithAprilTagCommand> alignToReefWithAprilTagCommandProvider) {
        this.driveToReefFaceCommandProvider = driveToReefFaceCommandProvider;
        this.alignToReefWithAprilTagCommandProvider = alignToReefWithAprilTagCommandProvider;
    }

    public SequentialCommandGroup create(Landmarks.ReefFace targetReefFace, Landmarks.Branch branch) {
        var group = new SequentialCommandGroup();

        var driveToReefFace = driveToReefFaceCommandProvider.get();
        driveToReefFace.setTargetReefFacePose(targetReefFace);
        var alignToReefWithAprilTag = alignToReefWithAprilTagCommandProvider.get();
        alignToReefWithAprilTag.setYOFFset(branch);

        group.addCommands(driveToReefFace, alignToReefWithAprilTag);

        return group;
    }
}
