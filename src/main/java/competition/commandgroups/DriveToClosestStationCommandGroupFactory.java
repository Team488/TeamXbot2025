package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.AlignToSpecificHumanLoadingStationCommand;
import competition.subsystems.drive.commands.DriveToCoralStationInterstitialCommand;
import competition.subsystems.drive.commands.ShoveCoralStationCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import javax.inject.Inject;
import javax.inject.Provider;
import xbot.common.command.NamedInstantCommand;

public class DriveToClosestStationCommandGroupFactory {
    private final Provider<DriveToCoralStationInterstitialCommand>
            driveToCoralStationSectionCommandProv;
    private final Provider<AlignToSpecificHumanLoadingStationCommand>
            alignToCoralStationCommandProv;
    private final Provider<IntakeUntilCoralCollectedCommand>
            intakeUntilCoralCollectedCommandProv;
    private final Provider<ShoveCoralStationCommand> shoveCoralStationCommandProv;
    private final PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    private final PoseSubsystem poseSubsystem;

    @Inject
    public DriveToClosestStationCommandGroupFactory(PoseSubsystem poseSubsystem,
                                                    Provider<DriveToCoralStationInterstitialCommand>
                                                            driveToCoralStationSectionCommandProv,
                                                    Provider<AlignToSpecificHumanLoadingStationCommand>
                                                            alignToCoralStationCommandProv,
                                                    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                    Provider<IntakeUntilCoralCollectedCommand>
                                                            intakeUntilCoralCollectedCommandProv,
                                                    Provider<ShoveCoralStationCommand> shoveCoralStationCommandProv) {
        this.driveToCoralStationSectionCommandProv =
                driveToCoralStationSectionCommandProv;
        this.alignToCoralStationCommandProv = alignToCoralStationCommandProv;
        this.prepCoralSystemCommandGroupFactory =
                prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommandProv =
                intakeUntilCoralCollectedCommandProv;
        this.shoveCoralStationCommandProv = shoveCoralStationCommandProv;
        this.poseSubsystem = poseSubsystem;
    }

    public SequentialCommandGroup createDriveOnly(boolean addPoint) {
        // Drive to coral station using terminal approach, have an interstitial
        // point if needed
        var driveToCoralStation = new SequentialCommandGroup();
        driveToCoralStation.setName("DriveToCoralStation");
        var alignToCoralStationCommand = alignToCoralStationCommandProv.get();
        var shoveCoralStationCommand = shoveCoralStationCommandProv.get();

        // We can add an interstitial point between scoring at the reef and
        // terminally approaching to the coral station to avoid rotating into the
        // reef
        var driveToCoralStationSectionCommand =
                driveToCoralStationSectionCommandProv.get();
        driveToCoralStation.addCommands(
                new NamedInstantCommand("GetNearestCoralStation", () -> {
                    Landmarks.CoralStation station =
                            poseSubsystem.getClosestCoralStation();
                    System.out.println("CLOSEST CORAL STATION: " + station);
                    if (addPoint) {
                        driveToCoralStationSectionCommand.setTargetCoralStationSection(
                                station);
                    }
                    alignToCoralStationCommand.setCoralStation(station);
                    shoveCoralStationCommand.setShoveAngle(station);
                }));
        if(addPoint) {
            driveToCoralStation.addCommands(
                    driveToCoralStationSectionCommand.withTimeout(2.0));
        }
        driveToCoralStation.addCommands(alignToCoralStationCommand);
        return driveToCoralStation.andThen(shoveCoralStationCommand.withTimeout(4));
    }

    public ParallelDeadlineGroup createWithIntakeUntilCollected(
            boolean addPoint) {
        // Overarching command group — preps coral system and drives to coral
        // station at the same time, command group stops if a coral is collected
        var driveUntilIntake =
                new ParallelDeadlineGroup(intakeUntilCoralCollectedCommandProv.get());
        driveUntilIntake.setName(
                "DriveToStationAndIntakeUntilCollectedCommandGroup");

        // Prep coral system to coral collection
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(
                () -> Landmarks.CoralLevel.COLLECTING);
        driveUntilIntake.addCommands(prepCoralSystem);

        // Drive to coral station using terminal approach, have an interstitial
        // point if needed
        var driveToCoralStation = new SequentialCommandGroup();
        driveToCoralStation.setName("DriveToCoralStation");
        var alignToCoralStationCommand = alignToCoralStationCommandProv.get();
        var shoveCoralStationCommand = shoveCoralStationCommandProv.get();

        // We can add an interstitial point between scoring at the reef and
        // terminally approaching to the coral station to avoid rotating into the
        // reef
        driveToCoralStation.addCommands(
                new NamedInstantCommand("GetNearestCoralStation", () -> {
                    Landmarks.CoralStation station =
                            poseSubsystem.getClosestCoralStation();
                    if (addPoint) {
                        var driveToCoralStationSectionCommand =
                                driveToCoralStationSectionCommandProv.get();
                        driveToCoralStationSectionCommand.setTargetCoralStationSection(
                                station);
                        driveToCoralStation.addCommands(
                                driveToCoralStationSectionCommand.withTimeout(2.0));
                    }
                    alignToCoralStationCommand.setCoralStation(station);
                    shoveCoralStationCommand.setShoveAngle(station);
                }));

        driveToCoralStation.addCommands(alignToCoralStationCommand);
        driveUntilIntake.addCommands(
                driveToCoralStation.andThen(shoveCoralStationCommand.withTimeout(4)));

        return driveUntilIntake;
    }
}
