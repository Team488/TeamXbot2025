package competition.subsystems.elevator.commands;

import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;
import java.util.function.Supplier;

public class SetElevatorTargetHeightCommand extends BaseSetpointCommand {

    ElevatorSubsystem elevator;
    private Supplier<Landmarks.CoralLevel> heightSupplier;

    @Inject
    public SetElevatorTargetHeightCommand(ElevatorSubsystem elevator){
        super(elevator);
        this.elevator = elevator;
    }

    public void setHeightSupplier(Supplier<Landmarks.CoralLevel> heightSupplier) {
        this.heightSupplier = heightSupplier;
    }

    public void setHeight(Landmarks.CoralLevel height) {
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
