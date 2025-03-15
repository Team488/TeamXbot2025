package competition.commandgroups.vision_path;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AlignToSpecificHumanLoadingStationCommand;
import competition.subsystems.drive.commands.DriveToCoralStationInterstitialCommand;
import competition.subsystems.drive.commands.ShoveCoralStationCommand;
import competition.subsystems.drive.commands.vision_path.PathToNearestCoralStationSectionCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.kobe.xbot.Utilities.Entities.XTableValues;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.function.Supplier;

public class PathToNearestStationAndIntakeUntilCollectedCommandGroupFactory {

    Provider<DriveToCoralStationInterstitialCommand> driveToCoralStationSectionCommandProv;
    Provider<AlignToSpecificHumanLoadingStationCommand> alignToCoralStationCommandProv;
    Provider<IntakeUntilCoralCollectedCommand> intakeUntilCoralCollectedCommandProv;
    Provider<ShoveCoralStationCommand> shoveCoralStationCommandProv;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    Provider<PathToNearestCoralStationSectionCommand>pathToNearestCoralStationSectionCommandProv;
    PoseSubsystem pose;
    CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;
    Logger log;
    DriveSubsystem driveSubsystem;


    @Inject
    public PathToNearestStationAndIntakeUntilCollectedCommandGroupFactory(Provider<DriveToCoralStationInterstitialCommand>
                                                                                      driveToCoralStationSectionCommandProv,
                                                                          Provider<AlignToSpecificHumanLoadingStationCommand>
                                                                                  alignToCoralStationCommandProv,
                                                                          PrepCoralSystemCommandGroupFactory
                                                                                      prepCoralSystemCommandGroupFactory,
                                                                          Provider<IntakeUntilCoralCollectedCommand>
                                                                                      intakeUntilCoralCollectedCommandProv,
                                                                          Provider<ShoveCoralStationCommand>
                                                                                      shoveCoralStationCommandProv,
                                                                          Provider<PathToNearestCoralStationSectionCommand>
                                                                                      pathToNearestCoralStationSectionCommandProv,
                                                                          PoseSubsystem pose,
                                                                          CoprocessorCommunicationSubsystem
                                                                                      coprocessorCommunicationSubsystem,
                                                                          DriveSubsystem driveSubsystem) {
        this.driveToCoralStationSectionCommandProv = driveToCoralStationSectionCommandProv;
        this.alignToCoralStationCommandProv = alignToCoralStationCommandProv;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommandProv = intakeUntilCoralCollectedCommandProv;
        this.shoveCoralStationCommandProv = shoveCoralStationCommandProv;
        this.pathToNearestCoralStationSectionCommandProv = pathToNearestCoralStationSectionCommandProv;
        this.pose = pose;
        this.coprocessorCommunicationSubsystem = coprocessorCommunicationSubsystem;
        this.log = LogManager.getLogger("PATHINGLOG");
        this.driveSubsystem = driveSubsystem;
    }

    public ParallelDeadlineGroup create(boolean addPoint) {
        Supplier<Landmarks.CoralStation> coralStationSupplier = () -> {

            var station = pose.getClosestCoralStation();
            log.info("initializing station");
            return station;
        };
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
            driveToCoralStationSectionCommand.setTargetCoralStationSupplier(coralStationSupplier);
            driveToCoralStation.addCommands(driveToCoralStationSectionCommand.withTimeout(2.0));
        }
        var alignToCoralStationCommand = alignToCoralStationCommandProv.get();
        var shoveCoralStationCommand = shoveCoralStationCommandProv.get();
        alignToCoralStationCommand.setTargetCoralStationSupplier(coralStationSupplier);

        driveToCoralStation.addCommands(alignToCoralStationCommand);
        shoveCoralStationCommand.setShoveAngleSupplier(coralStationSupplier);

        var pathOrDriveToStation = new SequentialCommandGroup();
        var pathToNearestStation = pathToNearestCoralStationSectionCommandProv.get();
        pathToNearestStation.setOptions(XTableValues.TraversalOptions.newBuilder()
                        .setMetersPerSecond(driveSubsystem.getMaxTargetSpeedMetersPerSecond())
                        .setAccelerationMetersPerSecond(driveSubsystem.getMaxAccelerationMetersPerSecondSquared())
                .build());
        var driveToStationAndShove = driveToCoralStation.andThen(shoveCoralStationCommand.withTimeout(4));
        pathOrDriveToStation.addCommands(pathToNearestStation);
        var fallBack = new ConditionalCommand(
                new WaitCommand(0), driveToStationAndShove, () -> {
                    var isHealthy  = coprocessorCommunicationSubsystem.getIsCoprocessorHealthy();
                    log.info("isHealthy", isHealthy);
                    return isHealthy;
                }
        );
        pathOrDriveToStation.addCommands(fallBack);
        driveUntilIntake.addCommands(pathOrDriveToStation);

        return driveUntilIntake;
    }
}
