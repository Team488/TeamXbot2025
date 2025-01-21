package competition.subsystems;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.algae_collection.AlgaeCollectionSubsystem;
import competition.subsystems.algae_collection.commands.AlgaeCollectionStopCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.elevator.ElevatorMechanism;
import competition.subsystems.elevator.ElevatorMechanismDemoCommand;

/**
 * For setting the default commands on subsystems
 */
@Singleton
public class SubsystemDefaultCommandMap {

    @Inject
    public SubsystemDefaultCommandMap() {}

    @Inject
    public void setupDriveSubsystem(DriveSubsystem driveSubsystem, SwerveDriveWithJoysticksCommand command) {
        // temp disable drive to use gamepad for elevator demo
        //driveSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupAlgaeCollectionSubsystem(AlgaeCollectionSubsystem algaeCollectionSubsystem, AlgaeCollectionStopCommand command) {
        algaeCollectionSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupElevatorMechanism(ElevatorMechanism elevatorMechanism, ElevatorMechanismDemoCommand command) {
        elevatorMechanism.setDefaultCommand(command);
    }
}
