package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.command.BaseSubsystem;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class ElevatorMaintainerCommand extends BaseCommand {

    public enum MaintainerMode{
        Calibrating,
        GaveUp,
        Calibrated,
    }

    ElevatorSubsystem elevator;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, PropertyFactory pf){

    }

    @Override
    public void initialize() {
        log.info("initializing");
    }

    @Override
    public void execute(){

    }

}
