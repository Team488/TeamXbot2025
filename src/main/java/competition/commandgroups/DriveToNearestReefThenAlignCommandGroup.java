package competition.commandgroups;

import competition.subsystems.drive.commands.AlignToReefWithAprilTagCommand;
import competition.subsystems.drive.commands.DriveToNearestReefFaceUntilDetectionCommand;
import xbot.common.command.BaseSequentialCommandGroup;

import javax.inject.Inject;

public class DriveToNearestReefThenAlignCommandGroup extends BaseSequentialCommandGroup {

    // TODO: update to new AlignToReefWithAprilTagCommand
    @Inject
    public DriveToNearestReefThenAlignCommandGroup(DriveToNearestReefFaceUntilDetectionCommand driveToNearestReefFaceUntilDetectionCommand,
                                                   AlignToReefWithAprilTagCommand alignToReefWithAprilTagCommand) {

        this.addCommands(driveToNearestReefFaceUntilDetectionCommand);
        this.addCommands(alignToReefWithAprilTagCommand);
    }
}
