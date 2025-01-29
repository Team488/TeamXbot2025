package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class RiseToL4Command extends BaseCommand {

    ElevatorSubsystem elevator;

    @Inject
    public RiseToL4Command(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        log.info("initializing...");
        elevator.setTargetHeight(ElevatorSubsystem.ElevatorGoals.ScoreL4);

    }

    @Override
    public void execute() {
        elevator.setTargetHeight(ElevatorSubsystem.ElevatorGoals.ScoreL4);
    }
}
