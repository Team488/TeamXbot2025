package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.DriveToCoralStationSectionCommand;
import competition.subsystems.drive.commands.ShoveCoralStationCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class DriveToStationAndIntakeUntilCollectedCommandGroupFactory {

    DriveToCoralStationSectionCommand driveToCoralStationSectionCommand;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    ShoveCoralStationCommand shoveCoralStationCommand;
    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand;


    @Inject
    public DriveToStationAndIntakeUntilCollectedCommandGroupFactory(DriveToCoralStationSectionCommand driveToCoralStationSectionCommand,
                                                                    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                                    ShoveCoralStationCommand shoveCoralStationCommand,
                                                                    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand) {
        this.driveToCoralStationSectionCommand = driveToCoralStationSectionCommand;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.shoveCoralStationCommand = shoveCoralStationCommand;
        this.intakeUntilCoralCollectedCommand = intakeUntilCoralCollectedCommand;
    }

    public ParallelDeadlineGroup create(Landmarks.CoralStation station,
                                         Landmarks.CoralStationSection section,
                                         boolean addPoint) {
        var driveThenShove = new SequentialCommandGroup();

        var driveToCoralStationSectionWhilePrepping = new ParallelCommandGroup();
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        driveToCoralStationSectionCommand.setTargetCoralStationSection(station, section, addPoint);
        driveToCoralStationSectionWhilePrepping.addCommands(driveToCoralStationSectionCommand, prepCoralSystem);

        shoveCoralStationCommand.setShoveAngle(station);

        driveThenShove.addCommands(driveToCoralStationSectionWhilePrepping, shoveCoralStationCommand);

        return new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand, driveThenShove);

    }
}
