package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class LowerElevatorCommand extends BaseCommand {

    ElevatorSubsystem elevator;

    @Inject
    public LowerElevatorCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        log.info("initializing");
    }

    @Override
    public void execute(){
        elevator.lower();
    }
}


