package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class SetElevatorTargetHeightCommand extends BaseCommand {

    Double height;
    ElevatorSubsystem elevator;

    @Inject
    public SetElevatorTargetHeightCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        log.info("initializing");
        elevator.setTargetValue(height);
    }

    @Override
    public void execute() {
        elevator.setTargetValue(height);
    }

    public void setHeight(double height) {
        this.height = height;
    }
}
