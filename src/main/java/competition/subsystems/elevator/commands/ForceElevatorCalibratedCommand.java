package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

//This command exists entirely for testing purposes
public class ForceElevatorCalibratedCommand extends BaseCommand {

    ElevatorSubsystem elevator;

    @Inject
    public ForceElevatorCalibratedCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void initialize() {
        log.info("Initializing..");
        elevator.setCalibrated(true);
        elevator.setTargetValue(elevator.getCurrentValue());
        this.setRunsWhenDisabled(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
