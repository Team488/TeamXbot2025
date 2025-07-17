package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class MoveElevatorWithJoysticksCommand extends BaseCommand {
    final OperatorInterface oi;
    final ElevatorSubsystem elevatorSubsystem;

    @Inject
    MoveElevatorWithJoysticksCommand(OperatorInterface oi, ElevatorSubsystem elevatorSubsystem) {
        this.oi = oi;
        this.elevatorSubsystem = elevatorSubsystem;
        this.addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        double force = oi.operatorGamepad.getLeftStickY();
        System.out.println(force);
        elevatorSubsystem.setPower(force);
    }
}
