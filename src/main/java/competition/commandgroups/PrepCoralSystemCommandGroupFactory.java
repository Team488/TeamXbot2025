package competition.commandgroups;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_arm_pivot.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class PrepCoralSystemCommandGroupFactory {

    SetElevatorTargetHeightCommand setElevatorTargetHeightCommand;
    SetCoralArmTargetAngleCommand setCoralArmTargetAngleCommand;


    @Inject
    public PrepCoralSystemCommandGroupFactory(SetElevatorTargetHeightCommand setElevatorTargetHeightCommand,
                                              SetCoralArmTargetAngleCommand setCoralArmTargetAngleCommand) {
        this.setElevatorTargetHeightCommand = setElevatorTargetHeightCommand;
        this.setCoralArmTargetAngleCommand = setCoralArmTargetAngleCommand;
    }

    public SequentialCommandGroup create(ElevatorSubsystem.ElevatorGoals elevatorTargetHeight, CoralArmPivotSubsystem.ArmGoals armGoal) {
        var group = new SequentialCommandGroup();

        setElevatorTargetHeightCommand.setHeight(elevatorTargetHeight);
        // TODO: setCoralArmTargetAngleCommand.setAngle();

        group.addCommands(setElevatorTargetHeightCommand, setCoralArmTargetAngleCommand);

        return group;
    }


}
