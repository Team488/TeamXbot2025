package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.DriveToCoralStationSectionCommand;
import competition.subsystems.drive.commands.PathToCoralStationSectionCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import javax.inject.Inject;

public class PathToCoralStationAndIntake {
    DriveToCoralStationSectionCommand driveToCoralStationSectionCommand;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand;


    @Inject
    public PathToCoralStationAndIntake(PathToCoralStationSectionCommand pathToCoralStationSectionCommand,
                                       PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                       IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand) {
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommand = intakeUntilCoralCollectedCommand;
    }

    public ParallelDeadlineGroup create(Landmarks.CoralStation station,
                                        Landmarks.CoralStationSection section) {
        var driveToCoralStationSectionWhilePrepping = new ParallelCommandGroup();
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        driveToCoralStationSectionCommand.setTargetCoralStationSection(station, section);
        driveToCoralStationSectionWhilePrepping.addCommands(driveToCoralStationSectionCommand, prepCoralSystem);

        return new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand, driveToCoralStationSectionWhilePrepping);

    }
}
