package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.AlignToSpecificHumanLoadingStationCommand;
import competition.subsystems.drive.commands.DriveToCoralStationInterstitialCommand;
import competition.subsystems.drive.commands.ShoveCoralStationCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class DriveToStationAndIntakeUntilCollectedCommandGroupFactory {

    Provider<DriveToCoralStationInterstitialCommand> driveToCoralStationSectionCommandProv;
    Provider<AlignToSpecificHumanLoadingStationCommand> alignToCoralStationCommandProv;
    Provider<IntakeUntilCoralCollectedCommand> intakeUntilCoralCollectedCommandProv;
    Provider<ShoveCoralStationCommand> shoveCoralStationCommandProv;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;

    @Inject
    public DriveToStationAndIntakeUntilCollectedCommandGroupFactory(Provider<DriveToCoralStationInterstitialCommand> driveToCoralStationSectionCommandProv,
                                                                    Provider<AlignToSpecificHumanLoadingStationCommand> alignToCoralStationCommandProv,
                                                                    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                                    Provider<IntakeUntilCoralCollectedCommand> intakeUntilCoralCollectedCommandProv) {
        this.driveToCoralStationSectionCommandProv = driveToCoralStationSectionCommandProv;
        this.alignToCoralStationCommandProv = alignToCoralStationCommandProv;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommandProv = intakeUntilCoralCollectedCommandProv;
    }

    public ParallelDeadlineGroup create(Landmarks.CoralStation station,
                                         boolean addPoint) {
        // Overarching command group — preps coral system and drives to coral station at the same time, command group stops if a coral is collected
        var driveUntilIntake = new ParallelDeadlineGroup(intakeUntilCoralCollectedCommandProv.get());

        // Prep coral system to coral collection
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        driveUntilIntake.addCommands(prepCoralSystem);

        // Drive to coral station using terminal approach, have an interstitial point if needed
        var driveToCoralStation = new SequentialCommandGroup();

        // We can add an interstitial point between scoring at the reef and terminally approaching to the coral station to avoid rotating into the reef
        if (addPoint) {
            var driveToCoralStationSectionCommand = driveToCoralStationSectionCommandProv.get();
            driveToCoralStationSectionCommand.setTargetCoralStationSection(station);
            driveToCoralStation.addCommands(driveToCoralStationSectionCommand.withTimeout(2.0));
        }

        var alignToCoralStationCommand = alignToCoralStationCommandProv.get();
        alignToCoralStationCommand.setCoralStation(station);
        driveToCoralStation.addCommands(alignToCoralStationCommand);

        var shoveCoralStationCommand = shoveCoralStationCommandProv.get();
        shoveCoralStationCommand.setShoveAngle(station);
        driveToCoralStation.addCommands(shoveCoralStationCommand);

        driveUntilIntake.addCommands(driveToCoralStation);

        return driveUntilIntake;
    }
}
