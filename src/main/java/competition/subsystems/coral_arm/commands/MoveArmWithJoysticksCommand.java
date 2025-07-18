package competition.subsystems.coral_arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class MoveArmWithJoysticksCommand extends BaseCommand {
    final OperatorInterface oi;
    final CoralArmSubsystem coralArmSubsystem;

    final DoubleProperty counterGravity;

    @Inject
    MoveArmWithJoysticksCommand(OperatorInterface oi, CoralArmSubsystem coralArmSubsystem, PropertyFactory propertyFactory) {
        this.oi = oi;
        this.coralArmSubsystem = coralArmSubsystem;

        this.addRequirements(coralArmSubsystem);
        propertyFactory.setPrefix(this);
        this.counterGravity = propertyFactory.createPersistentProperty("Counter Arm Gravity", 0.1);
    }

    @Override
    public void execute() {
        double force = oi.operatorGamepad.getRightStickY();
        coralArmSubsystem.setPower(MathUtils.deadband(force, this.oi.getOperatorGamepadTypicalDeadband(),
                (value) -> MathUtils.exponentAndRetainSign(value, 2)
        ));
    }
}
