package competition.commandgroups;

import competition.subsystems.drive.commands.DriveToNearestReefFaceCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class DriveToRemoveNearestAlgaeCommandGroup extends SequentialCommandGroup {
    @Inject
    public DriveToRemoveNearestAlgaeCommandGroup(DriveToNearestReefFaceCommand driveToNearestReefFaceCommand) {
        this.addCommands(driveToNearestReefFaceCommand);
    }
}
