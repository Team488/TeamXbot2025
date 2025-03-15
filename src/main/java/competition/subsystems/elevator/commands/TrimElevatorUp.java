package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Inches;

public class TrimElevatorUp extends BaseCommand {
    final ElevatorSubsystem elevator;
    final OperatorInterface oi;

    @Inject
    public TrimElevatorUp(ElevatorSubsystem elevatorSubsystem, OperatorInterface operatorInterface){
        this.elevator = elevatorSubsystem;
        this.oi= operatorInterface;

    }

    public void initialize(){
        var oldTrim = elevator.trimValue.get().in(Inches);
        elevator.trimElevatorUp();
        log.info("Updating trim value from " + oldTrim + "into" + elevator.trimValue.get().in(Inches));
    }

    @Override
    public boolean isFinished(){
        return true;

    }
}
