package competition.subsystems.coral_arm.commands;

import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class CoralArmMaintainerCommand extends BaseMaintainerCommand<Angle> {

    CoralArmSubsystem coralArm;
    AlgaeArmSubsystem algaeArm;
    ElevatorSubsystem elevator;

    OperatorInterface oi;
    final DoubleProperty humanMaxPower;
    final DoubleProperty humanMinPower;

    final DoubleProperty algaeArmCollisionAngleDegrees;
    final DoubleProperty maxSafeArmAngleDegrees;

    final TrapezoidProfileManager profileManager;

    @Inject
    public CoralArmMaintainerCommand(CoralArmSubsystem armPivotSubsystem, ElevatorSubsystem elevator,
            AlgaeArmSubsystem algaeArm, PropertyFactory pf,
            HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
            TrapezoidProfileManager.Factory trapzoidProfileManagerFactory,
            OperatorInterface oi) {

        super(armPivotSubsystem, pf, hvmFactory, 1, 1);
        this.coralArm = armPivotSubsystem;
        this.algaeArm = algaeArm;
        this.elevator = elevator;

        this.oi = oi;
        pf.setPrefix(this);
        profileManager = trapzoidProfileManagerFactory.create(getPrefix() + "trapezoidMotion",
                60, 100, armPivotSubsystem.getCurrentValue().in(Degrees));
        pf.setDefaultLevel(Property.PropertyLevel.Important);

        humanMaxPower = pf.createPersistentProperty("HumanMaxPower", .20);
        humanMinPower = pf.createPersistentProperty("HumanMinPower", -.20);
        algaeArmCollisionAngleDegrees = pf.createPersistentProperty("AlgaeArmCollisionAngleDegrees", 90);
        maxSafeArmAngleDegrees = pf.createPersistentProperty("MaxSafeArmAngleDegrees", 60);

        decider.setDeadband(0.02);
    }

    @Override
    public void initialize() {
        super.initialize();
        setpoint = coralArm.getCurrentValue().in(Degrees);
    }

    @Override
    protected void coastAction() { // rest when no human control and before pid
        coralArm.setPower(0);
    }

    double setpoint = 0;

    @Override
    protected void calibratedMachineControlAction() { // manages and runs pid
        // if the arm is being requested to go to a position that would cause a
        // collision,
        // move to a safe position instead until that changes
        var currentTarget = wouldCollideWithAlgaeArm(coralArm.getTargetValue())
                ? Degrees.of(this.maxSafeArmAngleDegrees.get())
                : coralArm.getTargetValue();

        profileManager.setTargetPosition(
                currentTarget.in(Degrees),
                coralArm.getCurrentValue().in(Degree),
                coralArm.getCurrentVelocity().in(DegreesPerSecond),
                setpoint);
        setpoint = profileManager.getRecommendedPositionForTime();

        aKitLog.record("coralArmProfileTarget", setpoint);

        coralArm.setPositionalGoalIncludingOffset(Degrees.of(setpoint));
    }

    private boolean wouldCollideWithAlgaeArm(Angle targetGoal) {
        var coralArmGoalAboveSafeLevel = targetGoal.gt(Degrees.of(this.maxSafeArmAngleDegrees.get()));
        var algaeArmRaised = algaeArm.getCurrentValue().in(Degrees) >= algaeArmCollisionAngleDegrees.get();
        var elevatorBelowLevel2Height = elevator.getCurrentValue().lt(elevator.humanLoadHeight.get());

        return coralArmGoalAboveSafeLevel && algaeArmRaised && elevatorBelowLevel2Height;
    }

    @Override
    protected double getErrorMagnitude() {
        // distance from goal
        Angle currentAngle = coralArm.getCurrentValue();
        Angle targetAngle = coralArm.getTargetValue();
        Angle error = targetAngle.minus(currentAngle);

        return error.in(Degrees);
    }

    @Override
    protected double getHumanInput() {
        // gamepad controls: Left joy stick up/down & Left bumper to switch between
        // elevator/arm
        return MathUtils.constrainDouble(
                MathUtils.deadband(
                        oi.superstructureGamepad.getRightStickY(),
                        oi.getOperatorGamepadTypicalDeadband(),
                        (a) -> MathUtils.exponentAndRetainSign(a, 3)),
                humanMinPower.get(), humanMaxPower.get());
    }

    @Override
    protected double getHumanInputMagnitude() { 
        // turns values into absolute value
        return getHumanInput();
    }
}
