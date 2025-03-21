package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.AlignToSpecificHumanLoadingStationCommand;
import competition.subsystems.drive.commands.DriveToCoralStationInterstitialCommand;
import competition.subsystems.drive.commands.ShoveCoralStationCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.command.NamedInstantCommand;

import javax.inject.Inject;
import javax.inject.Provider;

public class DriveToClosestStationAndIntakeUntilCollectedCommandGroupFactory {

    Provider<DriveToCoralStationInterstitialCommand> driveToCoralStationSectionCommandProv;
    Provider<AlignToSpecificHumanLoadingStationCommand> alignToCoralStationCommandProv;
    Provider<IntakeUntilCoralCollectedCommand> intakeUntilCoralCollectedCommandProv;
    Provider<ShoveCoralStationCommand> shoveCoralStationCommandProv;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    PoseSubsystem poseSubsystem;

    @Inject
    public DriveToClosestStationAndIntakeUntilCollectedCommandGroupFactory(
            PoseSubsystem poseSubsystem,
            Provider<DriveToCoralStationInterstitialCommand> driveToCoralStationSectionCommandProv,
                                                                           Provider<AlignToSpecificHumanLoadingStationCommand> alignToCoralStationCommandProv,
                                                                           PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                                           Provider<IntakeUntilCoralCollectedCommand> intakeUntilCoralCollectedCommandProv,
                                                                           Provider<ShoveCoralStationCommand> shoveCoralStationCommandProv) {
        this.driveToCoralStationSectionCommandProv = driveToCoralStationSectionCommandProv;
        this.alignToCoralStationCommandProv = alignToCoralStationCommandProv;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommandProv = intakeUntilCoralCollectedCommandProv;
        this.shoveCoralStationCommandProv = shoveCoralStationCommandProv;
        this.poseSubsystem = poseSubsystem;
    }

    public ParallelDeadlineGroup create(
                                         boolean addPoint) {
        // Overarching command group — preps coral system and drives to coral station at the same time, command group stops if a coral is collected
        var driveUntilIntake = new ParallelDeadlineGroup(intakeUntilCoralCollectedCommandProv.get());
        driveUntilIntake.setName("DriveToStationAndIntakeUntilCollectedCommandGroup");

        // Prep coral system to coral collection
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        driveUntilIntake.addCommands(prepCoralSystem);

        // Drive to coral station using terminal approach, have an interstitial point if needed
        var driveToCoralStation = new SequentialCommandGroup();
        driveToCoralStation.setName("DriveToCoralStation");
        var alignToCoralStationCommand = alignToCoralStationCommandProv.get();
        var shoveCoralStationCommand = shoveCoralStationCommandProv.get();

        // We can add an interstitial point between scoring at the reef and terminally approaching to the coral station to avoid rotating into the reef
        driveToCoralStation.addCommands(
                new NamedInstantCommand("GetNearestCoralStation", () -> {
                    Landmarks.CoralStation station = poseSubsystem.getClosestCoralStation();
                    if (addPoint) {
                        var driveToCoralStationSectionCommand = driveToCoralStationSectionCommandProv.get();
                        driveToCoralStationSectionCommand.setTargetCoralStationSection(station);
                        driveToCoralStation.addCommands(driveToCoralStationSectionCommand.withTimeout(2.0));
                    }
                    alignToCoralStationCommand.setCoralStation(station);
                    shoveCoralStationCommand.setShoveAngle(station);

                })
        );


        driveToCoralStation.addCommands(alignToCoralStationCommand);
        driveUntilIntake.addCommands(driveToCoralStation.andThen(shoveCoralStationCommand.withTimeout(4)));

        return driveUntilIntake;
    }
}
