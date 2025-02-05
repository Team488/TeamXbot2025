package competition.subsystems.elevator.commands;

import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.logic.CalibrationDecider;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import javax.inject.Inject;
import javax.inject.Provider;

public class ElevatorMaintainerCommand extends BaseMaintainerCommand<Distance> {

    private final OperatorInterface oi;

    private final PIDManager positionPID;

    private final ElevatorSubsystem elevator;

    CalibrationDecider calibrationDecider;

    final DoubleProperty humanMaxPowerGoingUp;
    final DoubleProperty humanMaxPowerGoingDown;

    final DoubleProperty gravityPIDConstantPower;

    final TrapezoidProfileManager profileManager;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, Provider<PropertyFactory> pfProvider,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     CalibrationDecider.CalibrationDeciderFactory calibrationDeciderFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi){
        super(elevator, pfProvider.get(),hvmFactory, Inches.of(1).in(Meters), 0.2);
        var pf = pfProvider.get();
        pf.setPrefix(this);
        this.elevator = elevator;
        profileManager = new TrapezoidProfileManager(getPrefix() + "trapezoidMotion", pfProvider.get(), 1, 1, elevator.getCurrentValue().in(Meters));

        this.oi = oi;

        calibrationDecider = calibrationDeciderFactory.create("calibrationDecider");
        calibrationDecider.reset();

        positionPID = pidf.create(getPrefix() + "positionPID", 5.0, 0, 0.5);

        this.humanMaxPowerGoingUp = pf.createPersistentProperty("maxPowerGoingUp", 1);
        this.humanMaxPowerGoingDown = pf.createPersistentProperty("maxPowerGoingDown", -0.2);

        this.gravityPIDConstantPower = pf.createPersistentProperty("gravityPIDConstant", 0.015);
    }

    @Override
    public void initialize() {
        log.info("initializing");
        calibrationDecider.reset();
    }

    @Override
    protected void coastAction() {
        elevator.setPower(0);
    }

    @Override
    protected void calibratedMachineControlAction() {
        profileManager.setTargetPosition(
            elevator.getTargetValue().in(Meters),
            elevator.getCurrentValue().in(Meters),
            elevator.getCurrentVelocity().in(MetersPerSecond)
        );
        var setpoint = profileManager.getRecommendedPositionForTime();

        // it's helpful to log this to know where the robot is actually trying to get to in the moment
        aKitLog.record("elevatorProfileTarget", setpoint);

        //handles pidding via motor controller and setting power to elevator
        elevator.masterMotor.setPositionTarget(Degrees.of(setpoint * elevator.rotationsPerMeter.get() * 360), XCANMotorController.MotorPidMode.TrapezoidalVoltage);
    }

    @Override
    protected void uncalibratedMachineControlAction() {
        var mode = calibrationDecider.decideMode(elevator.isCalibrated());

        switch (mode){
            case Calibrated -> calibratedMachineControlAction();
            case Attempting -> attemptCalibration();
            case GaveUp -> humanControlAction();
            default -> humanControlAction();
        }
    }

    private void attemptCalibration(){
        elevator.setPower(elevator.calibrationNegativePower.get());

        if (elevator.isTouchingBottom()){
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
        return MathUtils.constrainDouble(
                MathUtils.deadband(
                    oi.superstructureGamepad.getLeftVector().getY(),
                    oi.getOperatorGamepadTypicalDeadband(),
                    (a) -> (a)),
                humanMaxPowerGoingDown.get(), humanMaxPowerGoingUp.get());
    }

    @Override
    protected double getHumanInputMagnitude() {
        return Math.abs(getHumanInput());
    }



}
