package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class MoveElevatorWithJoysticksCommand extends BaseCommand {
    final OperatorInterface oi;
    final ElevatorSubsystem elevatorSubsystem;

    final DoubleProperty counterGravity;

    @Inject
    MoveElevatorWithJoysticksCommand(OperatorInterface oi, ElevatorSubsystem elevatorSubsystem, PropertyFactory propertyFactory) {
        this.oi = oi;
        this.elevatorSubsystem = elevatorSubsystem;
        this.addRequirements(elevatorSubsystem);
        propertyFactory.setPrefix(this);

        this.counterGravity = propertyFactory.createPersistentProperty("Counter Elevator Gravity", 0.1);
    }

    @Override
    public void execute() {
        double force = oi.operatorGamepad.getLeftStickY();
        elevatorSubsystem.setPower(MathUtils.deadband(force, this.oi.getOperatorGamepadTypicalDeadband(),
                (value) -> MathUtils.exponentAndRetainSign(value, 2)
        ) + counterGravity.get());
    }
}
