package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class SetUseDistanceSensorCommand extends BaseCommand {

    final ElevatorSubsystem elevator;
    private boolean value;

    @Inject
    public SetUseDistanceSensorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    public void setValue(boolean value) {
        this.value = value;
    }

    @Override
    public void initialize() {
        elevator.setUseDistanceSensor(value);
    }

    @Override
    public void execute() {
        // No-code
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
