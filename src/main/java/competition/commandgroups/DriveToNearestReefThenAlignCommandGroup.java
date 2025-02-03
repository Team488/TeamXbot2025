package competition.commandgroups;

import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DriveToNearestReefFaceCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class DriveToNearestReefThenAlignCommandGroup extends SequentialCommandGroup {

    @Inject
    public DriveToNearestReefThenAlignCommandGroup(DriveToNearestReefFaceCommand driveToNearestReefFaceCommand,
                                                   AlignToReefWithAprilTagCommand alignToReefWithAprilTagCommand) {

        this.addCommands(driveToNearestReefFaceCommand);
        this.addCommands(alignToReefWithAprilTagCommand);
    }
}
