package competition.subsystems.coral_arm.commands;

import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;


public class CoralArmMaintainerCommand extends BaseMaintainerCommand<Angle> {

   CoralArmSubsystem coralArm;
   OperatorInterface oi;
   final DoubleProperty humanMaxPower;
   final DoubleProperty humanMinPower;

   final TrapezoidProfileManager profileManager;

   @Inject
   public CoralArmMaintainerCommand(CoralArmSubsystem armPivotSubsystem, PropertyFactory pf,
                                    HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                    TrapezoidProfileManager.Factory trapzoidProfileManagerFactory,
                                    OperatorInterface oi) {

       super(armPivotSubsystem, pf, hvmFactory, 1,1);
       this.coralArm = armPivotSubsystem;
       this.oi = oi;
       pf.setPrefix(this);
       profileManager = trapzoidProfileManagerFactory.create(getPrefix() + "trapezoidMotion",
               90, 90, armPivotSubsystem.getCurrentValue().in(Degrees));
       pf.setDefaultLevel(Property.PropertyLevel.Important);

       humanMaxPower = pf.createPersistentProperty("HumanMaxPower", .20);
       humanMinPower = pf.createPersistentProperty("HumanMinPower", -.20);
   }

    @Override
    protected void coastAction() { //rest when no human control and before pid
        coralArm.setPower(0);
    }


    @Override
    protected void calibratedMachineControlAction() { //manages and runs pid
        profileManager.setTargetPosition(
            coralArm.getTargetValue().in(Degrees),
            coralArm.getCurrentValue().in(Degrees),
            coralArm.getCurrentVelocity().in(DegreesPerSecond)
        );
        var setpoint = profileManager.getRecommendedPositionForTime();

        aKitLog.record("coralArmProfileTarget", setpoint);

        coralArm.armMotor.setPositionTarget(
                Rotations.of(setpoint * coralArm.getRotationsPerDegree()),
                XCANMotorController.MotorPidMode.Voltage);
    }

    @Override
    protected double getErrorMagnitude() { //distance from goal
       Angle currentAngle = coralArm.getCurrentValue();
       Angle targetAngle = coralArm.getTargetValue();
       Angle error = targetAngle.minus(currentAngle);

       return error.in(Degrees);
    }

    @Override
    protected double getHumanInput() { //gamepad controls: Left joy stick up/down & Left bumper to switch between elevator/arm
       return MathUtils.constrainDouble(
               MathUtils.deadband(
                       oi.superstructureGamepad.getRightStickY(),
                       oi.getOperatorGamepadTypicalDeadband(),
                       (a) -> (a)),
               humanMinPower.get(), humanMaxPower.get());
    }

    @Override
    protected double getHumanInputMagnitude() { //turns values into absolute value
        return getHumanInput();
    }
}
