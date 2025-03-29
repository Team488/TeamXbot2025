package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ToggleElevatorMotionMagicCommand extends BaseCommand {
    ElevatorSubsystem elevator;

    @Inject
    public ToggleElevatorMotionMagicCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        log.info("Initializing..");
        elevator.toggleMotionMagic();
        elevator.setTargetValue(elevator.getCurrentValue());
        this.setRunsWhenDisabled(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
