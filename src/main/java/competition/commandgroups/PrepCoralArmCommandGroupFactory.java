package competition.commandgroups;

import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class PrepCoralArmCommandGroupFactory {

    SetElevatorTargetHeightCommand setElevatorTargetHeightCommand;


    @Inject
    public PrepCoralArmCommandGroupFactory(SetElevatorTargetHeightCommand setElevatorTargetHeightCommand
                                        ) {
        this.setElevatorTargetHeightCommand = setElevatorTargetHeightCommand;
    }

    public SequentialCommandGroup create(ElevatorSubsystem.ElevatorGoals elevatorTargetHeight) {
        var group = new SequentialCommandGroup();

        setElevatorTargetHeightCommand.setHeight(elevatorTargetHeight);
        group.addCommands(setElevatorTargetHeightCommand);

        return group;
    }


}
