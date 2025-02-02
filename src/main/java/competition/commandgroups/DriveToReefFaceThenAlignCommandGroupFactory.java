package competition.commandgroups;

import competition.subsystems.drive.commands.DriveToNearestReefFaceCommand;
import competition.subsystems.drive.commands.DriveToReefFaceCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class DriveToReefFaceThenAlignCommandGroupFactory {

    Provider<DriveToReefFaceCommand> driveToReefFaceCommandProvider;
    Provider<DriveToNearestReefCommandGroup> driveToNearestReefCommandGroupProvider;

    @Inject
    public DriveToReefFaceThenAlignCommandGroupFactory(Provider<DriveToReefFaceCommand> driveToReefFaceCommandProvider,
                                                       Provider<DriveToNearestReefCommandGroup> driveToNearestReefCommandGroupProvider) {
        this.driveToReefFaceCommandProvider = driveToReefFaceCommandProvider;
        this.driveToNearestReefCommandGroupProvider = driveToNearestReefCommandGroupProvider;
    }

    public SequentialCommandGroup create(Pose2d targetReefFacePose) {
        var group = new SequentialCommandGroup();

        var driveToReefFace = driveToReefFaceCommandProvider.get();
        driveToReefFace.setTargetReefFacePose(targetReefFacePose);
        var driveToNearestFaceCommand = driveToNearestReefCommandGroupProvider.get();

        group.addCommands(driveToReefFace, driveToNearestFaceCommand);

        return group;
    }
}
