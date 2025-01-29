package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class RiseToL2Command extends BaseCommand {

    ElevatorSubsystem elevator;

    @Inject
    public RiseToL2Command(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        log.info("initializing...");
        elevator.setTargetHeight(ElevatorSubsystem.ElevatorGoals.ScoreL2);

    }

    @Override
    public void execute() {
        elevator.setTargetHeight(ElevatorSubsystem.ElevatorGoals.ScoreL2);
    }
}
