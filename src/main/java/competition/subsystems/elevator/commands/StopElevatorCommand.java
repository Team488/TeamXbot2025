package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class StopElevatorCommand extends BaseCommand {

    ElevatorSubsystem elevator;

    @Inject
    public StopElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        log.info("initializing");
    }

    @Override
    public void execute() {
        elevator.stop();
    }
}