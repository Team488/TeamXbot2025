package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.AlignToSpecificHumanLoadingStationCommand;
import competition.subsystems.drive.commands.DriveToCoralStationSectionInterstitialCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class DriveToStationAndIntakeUntilCollectedCommandGroupFactory {

    DriveToCoralStationSectionInterstitialCommand driveToCoralStationSectionCommand;
    AlignToSpecificHumanLoadingStationCommand alignToCoralStationCommand;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand;


    @Inject
    public DriveToStationAndIntakeUntilCollectedCommandGroupFactory(DriveToCoralStationSectionInterstitialCommand driveToCoralStationSectionCommand,
                                                                    AlignToSpecificHumanLoadingStationCommand alignToCoralStationCommand,
                                                                    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                                    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand) {
        this.driveToCoralStationSectionCommand = driveToCoralStationSectionCommand;
        this.alignToCoralStationCommand = alignToCoralStationCommand;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommand = intakeUntilCoralCollectedCommand;
    }

    public ParallelDeadlineGroup create(Landmarks.CoralStation station,
                                         Landmarks.CoralStationSection section,
                                         boolean addPoint) {
        var driveUntilIntake = new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand);

        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);

        driveUntilIntake.addCommands(prepCoralSystem);

        var driveToCoralStation = new SequentialCommandGroup();

        if (addPoint) {
            driveToCoralStationSectionCommand.setTargetCoralStationSection(station, section);
            driveToCoralStation.addCommands(driveToCoralStationSectionCommand);
        }

        alignToCoralStationCommand.setCoralStation(station);
        driveToCoralStation.addCommands(alignToCoralStationCommand);

        driveUntilIntake.addCommands(driveToCoralStation);

        return driveUntilIntake;
    }
}
