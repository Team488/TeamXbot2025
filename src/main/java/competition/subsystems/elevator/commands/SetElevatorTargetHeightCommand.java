package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;
import java.util.function.Supplier;

public class SetElevatorTargetHeightCommand extends BaseSetpointCommand {

    ElevatorSubsystem elevator;
    private Supplier<ElevatorSubsystem.ElevatorGoals> heightSupplier;

    @Inject
    public SetElevatorTargetHeightCommand(ElevatorSubsystem elevator){
        super(elevator);
        this.elevator = elevator;
    }

    public void setHeightSupplier(Supplier<ElevatorSubsystem.ElevatorGoals> heightSupplier) {
        this.heightSupplier = heightSupplier;
    }

    public void setHeight(ElevatorSubsystem.ElevatorGoals height) {
        setHeightSupplier(() -> height);
    }

    @Override
    public void initialize() {
        log.info("initializing");
        elevator.setTargetHeight(this.heightSupplier.get());
    }

    @Override
    public void execute() {
        // No-op, wait for arms to get to the target
    }
}
