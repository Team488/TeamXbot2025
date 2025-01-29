package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class RiseToL1Command extends BaseCommand {

    ElevatorSubsystem elevator;

    @Inject
    public RiseToL1Command(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        log.info("initializing...");
        elevator.setTargetHeight(ElevatorSubsystem.ElevatorGoals.ScoreL1);

    }

    @Override
    public void execute() {
        elevator.setTargetHeight(ElevatorSubsystem.ElevatorGoals.ScoreL1);
    }
}
