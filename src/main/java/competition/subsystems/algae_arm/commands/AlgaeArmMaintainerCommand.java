package competition.subsystems.algae_arm.commands;

import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class AlgaeArmMaintainerCommand extends BaseMaintainerCommand<Angle> {
    final DoubleProperty humanMinPower;
    final DoubleProperty humanMaxPower;
    final AlgaeArmSubsystem algaeArm;
    final OperatorInterface oi;

    final TrapezoidProfileManager profileManager;

    @Inject
    public AlgaeArmMaintainerCommand(AlgaeArmSubsystem algaeArm, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvm,
                                     PIDManager.PIDManagerFactory pid, TrapezoidProfileManager.Factory trapzoidProfileManagerFactory,
                                     OperatorInterface oi) {
        super(algaeArm, pf, hvm, 3, 0.25);
        this.algaeArm = algaeArm;
        this.oi = oi;
        pf.setPrefix(this);
        pf.setDefaultLevel(Property.PropertyLevel.Important);
        humanMaxPower = pf.createPersistentProperty("HumanMaxPowerProperty", 1);
        humanMinPower = pf.createPersistentProperty("HumanMinPowerProperty", -.1);

        profileManager = trapzoidProfileManagerFactory.create(getPrefix() + "trapezoidMotion",
                60, 100, algaeArm.getCurrentValue().in(Degrees));

        decider.setDeadband(0.02);
    }


    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    protected void coastAction() {
        algaeArm.setPower(0);
    }

    @Override
    protected void calibratedMachineControlAction() { //manages and runs pid
        profileManager.setTargetPosition(
                algaeArm.getTargetValue().in(Degrees),
                algaeArm.getCurrentValue().in(Degree),
                algaeArm.getCurrentVelocity().in(DegreesPerSecond)
        );
        var setpoint = profileManager.getRecommendedPositionForTime();

        aKitLog.record("ProfileTarget", setpoint);

        algaeArm.setPositionalGoalIncludingOffset(Degrees.of(setpoint));
    }

    @Override
    protected double getErrorMagnitude() { //distance from goal
        Angle currentAngle = algaeArm.getCurrentValue();
        Angle targetAngle = algaeArm.getTargetValue();
        Angle error = targetAngle.minus(currentAngle);

        return error.in(Degrees);
    }

    @Override
    protected double getHumanInput() {
        if (oi.operatorGamepad.getXboxButton(XXboxController.XboxButton.Back).getAsBoolean()) {
            return MathUtils.constrainDouble(
                    MathUtils.deadband(
                            oi.operatorGamepad.getRightStickY(),
                            oi.getOperatorGamepadTypicalDeadband(),
                            (a) -> MathUtils.exponentAndRetainSign(a, 3)),
                    humanMinPower.get(), humanMaxPower.get());
        }
        return 0;
    }

    @Override
    protected double getHumanInputMagnitude() {
        return getHumanInput();
    }

}







