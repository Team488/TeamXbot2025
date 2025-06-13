package competition.commandgroups;

import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DriveToNearestReefFaceUntilDetectionCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class DriveToNearestReefThenAlignCommandGroup extends SequentialCommandGroup {

    // TODO: update to new AlignToReefWithAprilTagCommand
    @Inject
    public DriveToNearestReefThenAlignCommandGroup(DriveToNearestReefFaceUntilDetectionCommand driveToNearestReefFaceUntilDetectionCommand,
                                                   AlignToReefWithAprilTagCommand alignToReefWithAprilTagCommand) {

        this.addCommands(driveToNearestReefFaceUntilDetectionCommand);
        this.addCommands(alignToReefWithAprilTagCommand);
    }
}
