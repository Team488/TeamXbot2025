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
        aKitLog.record("OldTrimValue", elevator.trimValue.get().in(Inches));
        elevator.trimElevatorDown();
        aKitLog.record("NewTrimValue", elevator.trimValue.get().in(Inches));
    }

    @Override
    public boolean isFinished(){
        return true;

    }
}
