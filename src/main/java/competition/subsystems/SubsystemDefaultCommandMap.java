package competition.subsystems;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_arm.commands.MoveArmWithJoysticksCommand;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.coral_scorer.commands.DefaultCoralCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.MoveElevatorWithJoysticksCommand;

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

    @Inject
    public void setupElevatorSubsystem(ElevatorSubsystem elevatorSubsystem, MoveElevatorWithJoysticksCommand command) {
        elevatorSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupCoralArmSubsystem(CoralArmSubsystem coralArmSubsystem, MoveArmWithJoysticksCommand command) {
        coralArmSubsystem.setDefaultCommand(command);
    }
}
