package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;

public class RiseToL3Command extends BaseSetpointCommand {

    ElevatorSubsystem elevator;

    @Inject
    public RiseToL3Command(ElevatorSubsystem elevator){
        super(elevator);
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        log.info("initializing...");
        elevator.setTargetHeight(ElevatorSubsystem.ElevatorGoals.ScoreL3);

    }

    @Override
    public void execute() {
        elevator.setTargetHeight(ElevatorSubsystem.ElevatorGoals.ScoreL3);
    }
}
