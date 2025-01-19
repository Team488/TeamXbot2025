package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class SetElevatorTargetHeightCommand extends BaseCommand {

    Distance height;
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

    public void setHeight(Distance height) {
        this.height = height;
    }
}
