package competition.subsystems.coral_arm.commands;

import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;

public class CoralArmMaintainerCommand extends BaseMaintainerCommand<Angle> {

    final CoralArmSubsystem coralArm;
    final AlgaeArmSubsystem algaeArm;
    final ElevatorSubsystem elevator;

    final OperatorInterface oi;
    final DoubleProperty humanMaxPower;
    final DoubleProperty humanMinPower;

    final DoubleProperty algaeArmCollisionAngleDegrees;
    final DoubleProperty level123SafeArmAngleDegrees;
    final DoubleProperty level4SafeArmAngleDegrees;

    final TrapezoidProfileManager profileManager;

    final Alert collisionSafetiesEngaged = new Alert("Coral Arm: collision safeties engaged", Alert.AlertType.kWarning);

    @Inject
    public CoralArmMaintainerCommand(CoralArmSubsystem armPivotSubsystem, ElevatorSubsystem elevator,
                                     AlgaeArmSubsystem algaeArm, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     TrapezoidProfileManager.Factory trapzoidProfileManagerFactory,
                                     OperatorInterface oi) {
        super(armPivotSubsystem, pf, hvmFactory, 2, 0.10);
        this.coralArm = armPivotSubsystem;
        this.algaeArm = algaeArm;
        this.elevator = elevator;

        this.oi = oi;
        pf.setPrefix(this);
        profileManager = trapzoidProfileManagerFactory.create(getPrefix() + "trapezoidMotion",
                1500,
                1200,
                1000, //tune defaultMaxGap on real robot
                armPivotSubsystem.getCurrentValue().in(Degrees));
        pf.setDefaultLevel(Property.PropertyLevel.Important);

        humanMaxPower = pf.createPersistentProperty("HumanMaxPower", .20);
        humanMinPower = pf.createPersistentProperty("HumanMinPower", -.20);
        algaeArmCollisionAngleDegrees = pf.createPersistentProperty("AlgaeArmCollisionAngleDegrees", 90);
        level123SafeArmAngleDegrees = pf.createPersistentProperty("L123SafeArmAngleDegrees", 60);
        level4SafeArmAngleDegrees = pf.createPersistentProperty("L4SafeArmAngleDegrees", 126);

        decider.setDeadband(0.02);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    protected void coastAction() {
        // rest when no human control and before pid
        coralArm.setPower(0);
    }

    @Override
    protected void calibratedMachineControlAction() {
        // manages and runs pid
        // if the arm is being requested to go to a position that would cause a
        // collision, move to a safe position instead until that changes
        var imminentAlgaeArmCollision = wouldCollideWithAlgaeArm(coralArm.getTargetValue());
        var imminentReefCollision = wouldCollideWithReef(coralArm.getTargetValue());
        collisionSafetiesEngaged.set(imminentAlgaeArmCollision || imminentReefCollision);
        
        var currentTarget = coralArm.getTargetValue();
        if(imminentReefCollision) {
            currentTarget = Degrees.of(this.level4SafeArmAngleDegrees.get());
        }
        // this angle is even more conservative
        if(imminentAlgaeArmCollision) {
            currentTarget = Degrees.of(this.level123SafeArmAngleDegrees.get());
        }

        profileManager.setTargetPosition(
                currentTarget.in(Degrees),
                coralArm.getCurrentValue().in(Degree),
                coralArm.getCurrentVelocity().in(DegreesPerSecond));
        var setpoint = profileManager.getRecommendedPositionForTime();

        aKitLog.record("ProfileTarget", setpoint);

        coralArm.setPositionalGoalIncludingOffset(Degrees.of(setpoint));
    }

    private boolean wouldCollideWithAlgaeArm(Angle targetGoal) {
        var coralArmGoalAboveSafeLevel = targetGoal.gt(Degrees.of(this.level123SafeArmAngleDegrees.get()));
        var algaeArmRaised = algaeArm.getCurrentValue().in(Degrees) >= algaeArmCollisionAngleDegrees.get();
        var elevatorBelowLevel2Height = elevator.getCurrentValue().lt(elevator.l2Height.get().minus(Inches.of(0.25)));

        return coralArmGoalAboveSafeLevel && (algaeArmRaised || elevatorBelowLevel2Height);
    }

    private boolean wouldCollideWithReef(Angle targetGoal) {
        var coralArmGoalAboveSafeLevel = targetGoal.gt(Degrees.of(this.level4SafeArmAngleDegrees.get()));
        var elevatorBelowLevel2Height = elevator.getCurrentValue().lt(elevator.l4Height.get().minus(Inches.of(5)));
        return coralArmGoalAboveSafeLevel && elevatorBelowLevel2Height;
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
        if (!oi.operatorGamepad.getXboxButton(XXboxController.XboxButton.Back).getAsBoolean()) {
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
        // turns values into absolute value
        return getHumanInput();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            // Note - this is really important! We need to force the system out of onboard PID because otherwise,
            // on enable, the PID will have a brief moment of action where it tries to return to the position
            // it was at before being disabled.
            coralArm.setPower(0);
        }
    }
}
