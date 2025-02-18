package competition.subsystems.elevator.commands;

import competition.electrical_contract.ElectricalContract;
import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.logic.CalibrationDecider;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static xbot.common.logic.CalibrationDecider.CalibrationMode.GaveUp;

import javax.inject.Inject;

public class ElevatorMaintainerCommand extends BaseMaintainerCommand<Distance> {

    private final OperatorInterface oi;

    private final ElevatorSubsystem elevator;

    CalibrationDecider calibrationDecider;

    final DoubleProperty humanMaxPowerGoingUp;
    final DoubleProperty humanMaxPowerGoingDown;

    final DoubleProperty gravityPIDConstantPower;

    final TrapezoidProfileManager profileManager;

    final ElectricalContract contract;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     CalibrationDecider.CalibrationDeciderFactory calibrationDeciderFactory,
                                     TrapezoidProfileManager.Factory trapezoidProfileManagerFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi,
                                     ElectricalContract contract){
        super(elevator, pf, hvmFactory, Inches.of(1).in(Meters), 0.2);
        pf.setPrefix(this);
        this.elevator = elevator;
        profileManager = trapezoidProfileManagerFactory.create(
                getPrefix() + "trapezoidMotion",
                5,
                3.5,
                elevator.getCurrentValue().in(Meters));

        this.oi = oi;
        this.contract = contract;

        calibrationDecider = calibrationDeciderFactory.create("calibrationDecider");
        calibrationDecider.reset();

        this.humanMaxPowerGoingUp = pf.createPersistentProperty("humanMaxPowerGoingUp", 0.2);
        this.humanMaxPowerGoingDown = pf.createPersistentProperty("humanMaxPowerGoingDown", -0.2);

        this.gravityPIDConstantPower = pf.createPersistentProperty("gravityPIDConstant", 0.07416666);

        decider.setDeadband(0.02);
    }

    @Override
    public void initialize() {
        super.initialize();
        calibrationDecider.reset();
    }

    @Override
    protected void initializeMachineControlAction() {
        super.initializeMachineControlAction();
    }

    @Override
    protected void coastAction() {
        elevator.setPower(0);
    }

    @Override
    protected void calibratedMachineControlAction() {

        var currentValue = elevator.getCurrentValue();

        profileManager.setTargetPosition(
            elevator.getTargetValue().in(Meters),
            currentValue.in(Meters),
            elevator.getCurrentVelocity().in(MetersPerSecond)
        );
        var setpoint = profileManager.getRecommendedPositionForTime();

        var error = Meters.of(setpoint).minus(currentValue);
        elevator.setElevatorDeltaFromCurrentHeight(error);

        // it's helpful to log this to know where the robot is actually trying to get to in the moment
        aKitLog.record("elevatorProfileTarget", setpoint);
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
    protected double getHumanInputMagnitude() {
        return Math.abs(getHumanInput());
    }



}
