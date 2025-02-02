package competition.commandgroups;

import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DriveToNearestReefFaceCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class DriveToNearestReefCommandGroup extends SequentialCommandGroup {

    @Inject
    public DriveToNearestReefCommandGroup(DriveToNearestReefFaceCommand driveToNearestReefFaceCommand,
                                          AlignToReefWithAprilTagCommand alignToReefWithAprilTagCommand) {

        this.addCommands(driveToNearestReefFaceCommand);
        this.addCommands(alignToReefWithAprilTagCommand);
    }
}
