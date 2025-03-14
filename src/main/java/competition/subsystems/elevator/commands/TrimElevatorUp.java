package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class TrimElevatorUp extends BaseCommand {
    final ElevatorSubsystem elevator;
    final OperatorInterface oi;

    @Inject
    public TrimElevatorUp(ElevatorSubsystem elevatorSubsystem, OperatorInterface operatorInterface){
        this.elevator = elevatorSubsystem;
        this.oi= operatorInterface;
        this.addRequirements(elevator);

    }

    public void initialize(){
        elevator.trimElevatorUp();
    }

    @Override
    public boolean isFinished(){
        return true;

    }
}
