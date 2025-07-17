package competition.subsystems;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.coral_scorer.commands.DefaultCoralCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;

import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * For setting the default commands on subsystems
 */
@Singleton
public class SubsystemDefaultCommandMap {

    @Inject
    public SubsystemDefaultCommandMap() {
    }

    @Inject
    public void setupDriveSubsystem(DriveSubsystem driveSubsystem, SwerveDriveWithJoysticksCommand command) {
        driveSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupCoralScorerSubsystem(CoralScorerSubsystem coralScorerSubsystem, DefaultCoralCommand command) {
        coralScorerSubsystem.setDefaultCommand(command);
    }
}
