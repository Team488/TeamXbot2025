package competition.subsystems.coral_arm_pivot.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degrees;


public class CoralArmPivotMaintainerCommand extends BaseMaintainerCommand<Angle> {

   CoralArmPivotSubsystem armPivotSubsystem;
   OperatorInterface oi;
   final DoubleProperty humanMaxPower;
   final DoubleProperty humanMinPower;
   PIDManager positionalPID;

   @Inject
   public CoralArmPivotMaintainerCommand(CoralArmPivotSubsystem armPivotSubsystem, PropertyFactory pf,
                                         HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                         PIDManager.PIDManagerFactory pidf,
                                         OperatorInterface oi) {

       super(armPivotSubsystem, pf, hvmFactory, 1,1);
       this.armPivotSubsystem = armPivotSubsystem;
       this.oi = oi;
       pf.setPrefix(this);
       pf.setDefaultLevel(Property.PropertyLevel.Important);
       positionalPID = pidf.create(getPrefix() + "PositionalPID", 0,0,0);

       humanMaxPower = pf.createPersistentProperty("HumanMaxPower", .11);
       humanMinPower = pf.createPersistentProperty("HumanMinPower", -.11);
   }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    protected void coastAction() { //rest when no human control and before pid
        armPivotSubsystem.setPower(0);
    }


    @Override
    protected void calibratedMachineControlAction() { //manages and runs pid
        double power = positionalPID.calculate(armPivotSubsystem.getTargetValue().in(Degrees),
                armPivotSubsystem.getCurrentValue().in(Degrees));

        armPivotSubsystem.setPower(power);
        aKitLog.record("Target Angle", armPivotSubsystem.getTargetValue().in(Degrees));
        aKitLog.record("Current Angle", armPivotSubsystem.getCurrentValue().in(Degrees));
        aKitLog.record("Power", power);
    }

    @Override
    protected double getErrorMagnitude() { //distance from goal
       Angle currentAngle = armPivotSubsystem.getCurrentValue();
       Angle targetAngle = armPivotSubsystem.getTargetValue();
       Angle error = targetAngle.minus(currentAngle);

       return error.in(Degrees);
    }

    @Override
    protected double getHumanInput() { //gamepad controls: Left joy stick up/down & Left bumper to switch between elevator/arm
       return MathUtils.constrainDouble(
               MathUtils.deadband(
                       oi.programmerGamepad.getRightStickY(),
                       oi.getOperatorGamepadTypicalDeadband(),
                       (a) -> (a)),
               humanMinPower.get(), humanMaxPower.get());
    }

    @Override
    protected double getHumanInputMagnitude() { //turns values into absolute value
        return getHumanInput();
    }
}
