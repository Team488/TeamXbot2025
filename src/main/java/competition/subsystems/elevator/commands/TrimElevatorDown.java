package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Inches;

public class TrimElevatorDown extends BaseCommand {
    final ElevatorSubsystem elevator;
    final OperatorInterface oi;

    @Inject
    public TrimElevatorDown(ElevatorSubsystem elevatorSubsystem, OperatorInterface operatorInterface){
        this.elevator = elevatorSubsystem;
        this.oi = operatorInterface;

    }

    public void initialize(){

        var oldTrim = elevator.trimValue.get().in(Inches);
        elevator.trimElevatorDown();
        log.info("Updating trim value from" + oldTrim + "to" + elevator.trimValue.get().in(Inches));
        aKitLog.record("NewTrimValue", elevator.trimValue.get().in(Inches));
    }

    @Override
    public boolean isFinished(){
        return true;

    }
}
