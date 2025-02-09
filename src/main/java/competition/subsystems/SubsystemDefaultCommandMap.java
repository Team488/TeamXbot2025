package competition.subsystems;

import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import competition.subsystems.algae_arm.commands.AlgaeArmMaintainerCommand;
import competition.subsystems.algae_collection.AlgaeCollectionSubsystem;
import competition.subsystems.algae_collection.commands.AlgaeCollectionStopCommand;
import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_arm_pivot.commands.CoralArmPivotMaintainerCommand;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.coral_scorer.commands.StopCoralCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.ElevatorMaintainerCommand;

import javax.inject.Inject;
import javax.inject.Singleton;

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
    public void setupCoralScorerSubsystem(CoralScorerSubsystem subsystem, StopCoralCommand command) {
        subsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupElevatorSubsystem(ElevatorSubsystem elevatorSubsystem, ElevatorMaintainerCommand command){
        elevatorSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupCoralArmPivotSubsystem(CoralArmPivotSubsystem coralArmPivotSubsystem, CoralArmPivotMaintainerCommand command){
        coralArmPivotSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupAlgaeArmSubsystem(AlgaeArmSubsystem algaeArmSubsystem, AlgaeArmMaintainerCommand commands){
        algaeArmSubsystem.setDefaultCommand(commands);
    }
}