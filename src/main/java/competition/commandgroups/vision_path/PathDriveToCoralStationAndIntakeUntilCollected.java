package competition.commandgroups.vision_path;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToNearestCoralStationSectionCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import javax.inject.Inject;

public class PathDriveToCoralStationAndIntakeUntilCollected {
    PathDriveToNearestCoralStationSectionCommand driveToCoralStationSectionCommand;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand;


    @Inject
    public PathDriveToCoralStationAndIntakeUntilCollected(PathDriveToNearestCoralStationSectionCommand driveToCoralStationSectionCommand,
                                                          PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                          IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand) {
        this.driveToCoralStationSectionCommand = driveToCoralStationSectionCommand;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommand = intakeUntilCoralCollectedCommand;
    }

    public ParallelDeadlineGroup create() {
        var driveToCoralStationSectionWhilePrepping = new ParallelCommandGroup();
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        driveToCoralStationSectionWhilePrepping.addCommands(driveToCoralStationSectionCommand, prepCoralSystem);
        return new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand, driveToCoralStationSectionWhilePrepping);

    }

}
