package competition.subsystems;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.algae_collection.AlgaeCollectionSubsystem;
import competition.subsystems.algae_collection.commands.AlgaeCollectionStopCommand;
import competition.subsystems.arm_pivot.ArmPivotSubsystem;
import competition.subsystems.arm_pivot.commands.ArmPivotMaintainerCommand;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.coral_scorer.commands.StopCoralCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.ElevatorMaintainerCommand;
import competition.subsystems.humanLoadRamp.commands.HumanLoadRampRetractCommand;
import competition.subsystems.humanLoadRamp.HumanLoadRampSubsystem;

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
    public void setupArmPivotSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmPivotMaintainerCommand command) {
        armPivotSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupHumanLoadRampSubSystem(HumanLoadRampSubsystem humanLoadRampSubsystem, HumanLoadRampRetractCommand command){
        humanLoadRampSubsystem.setDefaultCommand(command);
    }


}
