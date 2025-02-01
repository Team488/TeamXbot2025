package competition.commandgroups;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_arm_pivot.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class PrepCoralSystemCommandGroupFactory {

    Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider;
    Provider<SetCoralArmTargetAngleCommand> setCoralArmTargetAngleCommandProvider;


    @Inject
    public PrepCoralSystemCommandGroupFactory(Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider,
                                              Provider<SetCoralArmTargetAngleCommand> setCoralArmTargetAngleCommandProvider) {
        this.setElevatorTargetHeightCommandProvider = setElevatorTargetHeightCommandProvider;
        this.setCoralArmTargetAngleCommandProvider = setCoralArmTargetAngleCommandProvider;
    }

    public SequentialCommandGroup create(ElevatorSubsystem.ElevatorGoals elevatorTargetHeight, CoralArmPivotSubsystem.ArmGoals armGoal) {
        var group = new SequentialCommandGroup();

        var setElevatorTargetHeightCommand = setElevatorTargetHeightCommandProvider.get();
        setElevatorTargetHeightCommand.setHeight(elevatorTargetHeight);
        var setCoralArmTargetAngleCommand = setCoralArmTargetAngleCommandProvider.get();
        setCoralArmTargetAngleCommand.setAngle(armGoal);

        group.addCommands(setElevatorTargetHeightCommand, setCoralArmTargetAngleCommand);

        return group;
    }


}
