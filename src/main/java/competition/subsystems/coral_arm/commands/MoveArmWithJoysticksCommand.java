package competition.subsystems.coral_arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;

import javax.inject.Inject;

public class MoveArmWithJoysticksCommand extends BaseCommand {
    final OperatorInterface oi;
    final CoralArmSubsystem coralArmSubsystem;

    @Inject
    MoveArmWithJoysticksCommand(OperatorInterface oi, CoralArmSubsystem coralArmSubsystem) {
        this.oi = oi;
        this.coralArmSubsystem = coralArmSubsystem;

        this.addRequirements(coralArmSubsystem);
    }

    @Override
    public void execute() {
        double force = oi.operatorGamepad.getRightStickY();
        coralArmSubsystem.setPower(MathUtils.deadband(force, this.oi.getOperatorGamepadTypicalDeadband(),
                (value) -> MathUtils.exponentAndRetainSign(value, 2)
        ));
    }
}
