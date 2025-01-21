package competition.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import javax.inject.Inject;

import competition.operator_interface.OperatorInterface;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;

public class ElevatorMechanismDemoCommand extends BaseCommand {
    final ElevatorMechanism mech;
    final OperatorInterface oi;

    @Inject
    public ElevatorMechanismDemoCommand(ElevatorMechanism mech, OperatorInterface oi) {
        this.mech = mech;
        this.oi = oi;
        this.addRequirements(mech);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double elevatorDelta = MathUtils.deadband(oi.gamepad.getLeftStickY(), 0.1);
        mech.elevatorHeight = mech.elevatorHeight.plus(Meters.of(elevatorDelta * 0.01));

        double armDelta = -MathUtils.deadband(oi.gamepad.getRightStickY(), 0.1);
        mech.armAngle = mech.armAngle.plus(Degrees.of(armDelta * 1));
    }


    
}
