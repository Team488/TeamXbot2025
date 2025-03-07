package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.AlignToSpecificHumanLoadingStationCommand;
import competition.subsystems.drive.commands.DriveToCoralStationInterstitialCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class DriveToStationAndIntakeUntilCollectedCommandGroupFactory {

    DriveToCoralStationInterstitialCommand driveToCoralStationSectionCommand;
    AlignToSpecificHumanLoadingStationCommand alignToCoralStationCommand;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand;


    @Inject
    public DriveToStationAndIntakeUntilCollectedCommandGroupFactory(DriveToCoralStationInterstitialCommand driveToCoralStationSectionCommand,
                                                                    AlignToSpecificHumanLoadingStationCommand alignToCoralStationCommand,
                                                                    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                                    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand) {
        this.driveToCoralStationSectionCommand = driveToCoralStationSectionCommand;
        this.alignToCoralStationCommand = alignToCoralStationCommand;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommand = intakeUntilCoralCollectedCommand;
    }

    public ParallelDeadlineGroup create(Landmarks.CoralStation station,
                                         boolean addPoint) {
        // Overarching command group — preps coral system and drives to coral station at the same time, command group stops if a coral is collected
        var driveUntilIntake = new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand);

        // Prep coral system to coral collection
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        driveUntilIntake.addCommands(prepCoralSystem);

        // Drive to coral station using terminal approach, have an interstitial point if needed
        var driveToCoralStation = new SequentialCommandGroup();
        // We can add an interstitial point between scoring at the reef and terminally approaching to the coral station to avoid rotating into the reef
        if (addPoint) {
            driveToCoralStationSectionCommand.setTargetCoralStationSection(station);
            driveToCoralStation.addCommands(driveToCoralStationSectionCommand.withTimeout(2.0));
        }
        alignToCoralStationCommand.setCoralStation(station);
        driveToCoralStation.addCommands(alignToCoralStationCommand);
        driveUntilIntake.addCommands(driveToCoralStation);

        return driveUntilIntake;
    }
}
