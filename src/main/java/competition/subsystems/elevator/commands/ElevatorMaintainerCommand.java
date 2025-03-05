package competition.subsystems.elevator.commands;

import competition.electrical_contract.ElectricalContract;
import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import xbot.common.advantage.AKitLogger;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.logic.CalibrationDecider;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.AngleProperty;
import xbot.common.properties.DistanceProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static xbot.common.logic.CalibrationDecider.CalibrationMode.GaveUp;

import javax.inject.Inject;

public class ElevatorMaintainerCommand extends BaseMaintainerCommand<Distance> {

    private final OperatorInterface oi;

    private final ElevatorSubsystem elevator;
    private final CoralArmSubsystem coralArm;

    CalibrationDecider calibrationDecider;

    final DoubleProperty humanMaxPowerGoingUp;
    final DoubleProperty humanMaxPowerGoingDown;

    final DoubleProperty gravityPIDConstantPower;

    final TrapezoidProfileManager.Factory trapezoidProfileManagerFactory;
    TrapezoidProfileManager profileManager;

    final ElectricalContract contract;
    final AngleProperty coralArmCollisionAngleDegrees;
    final DistanceProperty elevatorTargetChangingThreshold;

    final Alert collisionSafetiesEngaged = new Alert("Elevator: collision safeties engaged", Alert.AlertType.kWarning);

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, CoralArmSubsystem coralArm, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     CalibrationDecider.CalibrationDeciderFactory calibrationDeciderFactory,
                                     TrapezoidProfileManager.Factory trapezoidProfileManagerFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi,
                                     ElectricalContract contract){
        super(elevator, pf, hvmFactory, Inches.of(1.5).in(Meters), 0.1);
        pf.setPrefix(this);
        this.elevator = elevator;
        this.coralArm = coralArm;
        this.trapezoidProfileManagerFactory = trapezoidProfileManagerFactory;
        createNewProfileManager();

        this.oi = oi;
        this.contract = contract;

        calibrationDecider = calibrationDeciderFactory.create("calibrationDecider");
        calibrationDecider.reset();

        this.humanMaxPowerGoingUp = pf.createPersistentProperty("humanMaxPowerGoingUp", 0.2);
        this.humanMaxPowerGoingDown = pf.createPersistentProperty("humanMaxPowerGoingDown", -0.2);

        this.gravityPIDConstantPower = pf.createPersistentProperty("gravityPIDConstant", 0.07416666);
        this.coralArmCollisionAngleDegrees = pf.createPersistentProperty(
                "CoralArmCollisionAngleDegrees", Degrees.of(120));
        this.elevatorTargetChangingThreshold = pf.createPersistentProperty(
                "elevatorTargetChangingThresholdInches", Inches.of(3));

        decider.setDeadband(0.02);
    }

    private void createNewProfileManager(){
        profileManager = trapezoidProfileManagerFactory.create(
                getPrefix() + "trapezoidMotion",
                1, // 5 for competition
                1, // 3.5 for competition
                1000, //tune for real robot
                elevator.getCurrentValue().in(Meters));
    }

    @Override
    public void initialize() {
        super.initialize();
        calibrationDecider.reset();
        /*profileManager.resetState(
                elevator.getCurrentValue().in(Meters),
                elevator.getCurrentVelocity().in(MetersPerSecond));*/
        createNewProfileManager();
    }

    @Override
    protected void initializeMachineControlAction() {
        super.initializeMachineControlAction();
        /*profileManager.resetState(
                elevator.getCurrentValue().in(Meters),
                elevator.getCurrentVelocity().in(MetersPerSecond));*/
        createNewProfileManager();
    }

    @Override
    protected void coastAction() {
        elevator.setPower(0);
    }

    @Override
    protected void calibratedMachineControlAction() {

        var wouldCollide = wouldHitBranch(elevator.getTargetValue());
        var currentTarget = wouldCollide
                ? elevator.getCurrentValue()
                : elevator.getTargetValue();

        collisionSafetiesEngaged.set(wouldCollide);
        aKitLog.setLogLevel(AKitLogger.LogLevel.DEBUG);
        aKitLog.record("PM-TargetValue", currentTarget.in(Meters));
        aKitLog.record("PM-CurrentValue", elevator.getCurrentValue().in(Meters));
        aKitLog.record("PM-CurrentVelocity", elevator.getCurrentVelocity().in(MetersPerSecond));
        aKitLog.setLogLevel(AKitLogger.LogLevel.INFO);

        profileManager.setTargetPosition(
            currentTarget.in(Meters),
            elevator.getCurrentValue().in(Meters),
            elevator.getCurrentVelocity().in(MetersPerSecond)
        );
        var setpoint = profileManager.getRecommendedPositionForTime();
        aKitLog.record("elevatorProfileTarget", setpoint);

        elevator.setElevatorHeightGoalOnMotor(setpoint);
    }

    private boolean wouldHitBranch(Distance targetValue) {
        var coralArmAngleDangerous = coralArm.getCurrentValue().gt(coralArmCollisionAngleDegrees.get());
        var elevatorIsMoving = Math.abs(targetValue.minus(elevator.getCurrentValue()).in(Inches))
                > elevatorTargetChangingThreshold.get().in(Inches);
        return coralArmAngleDangerous && elevatorIsMoving;
    }

    //defaults humanControlAction if there is no bottom sensor
    @Override
    protected void uncalibratedMachineControlAction() {
        var mode = GaveUp;

        switch (mode){
            case Calibrated -> calibratedMachineControlAction();
            case Attempting -> attemptCalibration();
            default -> humanControlAction();
        }
    }

    private void attemptCalibration(){
        elevator.setPower(elevator.calibrationNegativePower.get());

        if (elevator.isTouchingBottom()) {
            elevator.markElevatorAsCalibratedAgainstLowerLimit();
            elevator.setTargetValue(elevator.getCurrentValue());
        }
    }

    @Override
    protected void humanControlAction() {
        super.humanControlAction();
    }

    //returns error magnitude of elevator in inches
    @Override
    protected double getErrorMagnitude() {
        var current = elevator.getCurrentValue();
        var target = elevator.getTargetValue();

        return Math.abs(target.in(Meters) - current.in(Meters));
    }

    @Override
    protected double getHumanInput() {

        double humanInput = MathUtils.constrainDouble(
                MathUtils.deadband(
                        oi.operatorGamepad.getLeftStickY(),
                        oi.getOperatorGamepadTypicalDeadband(),
                        (a) -> MathUtils.exponentAndRetainSign(a, 3)),
                humanMaxPowerGoingDown.get(), humanMaxPowerGoingUp.get());

        aKitLog.record("elevatorHumanInput", humanInput);
        return humanInput;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Note - this is really important! We need to force the system out of onboard PID because otherwise,
            // on enable, the PID will have a brief moment of action where it tries to return to the position
            // it was at before being disabled.
            elevator.setPower(0);
        }
    }

    @Override
    protected double getHumanInputMagnitude() {
        return Math.abs(getHumanInput());
    }



}
