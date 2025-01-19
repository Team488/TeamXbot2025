package competition.subsystems;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.algae_collection.AlgaeCollectionSubsystem;
import competition.subsystems.algae_collection.commands.AlgaeCollectionStopCommand;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.coral_scorer.commands.StopScoreCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;

/**
 * For setting the default commands on subsystems
 */
@Singleton
public class SubsystemDefaultCommandMap {

    @Inject
    public SubsystemDefaultCommandMap() {}

    @Inject
    public void setupDriveSubsystem(DriveSubsystem driveSubsystem, SwerveDriveWithJoysticksCommand command) {
        driveSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupAlgaeCollectionSubsystem(AlgaeCollectionSubsystem algaeCollectionSubsystem, AlgaeCollectionStopCommand command) {
        algaeCollectionSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupCoralScorerSubsystem(CoralScorerSubsystem subsystem, StopScoreCommand command) {
        subsystem.setDefaultCommand(command);
    }
}
