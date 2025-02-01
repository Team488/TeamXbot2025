package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;

public class SetElevatorTargetHeightCommand extends BaseSetpointCommand {

    ElevatorSubsystem.ElevatorGoals height;
    ElevatorSubsystem elevator;

    @Inject
    public SetElevatorTargetHeightCommand(ElevatorSubsystem elevator){
        super(elevator);
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        log.info("initializing");
        elevator.setTargetHeight(height);
    }

    @Override
    public void execute() {
        elevator.setTargetHeight(height);
    }

    public void setHeight(ElevatorSubsystem.ElevatorGoals height) {
        this.height = height;
    }
}
