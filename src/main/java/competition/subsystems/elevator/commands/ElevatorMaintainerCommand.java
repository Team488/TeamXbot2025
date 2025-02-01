package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import javax.inject.Inject;

public class ElevatorMaintainerCommand extends BaseMaintainerCommand<Distance> {

    public enum MaintainerMode{
        Calibrating,
        GaveUp,
        Calibrated,
    }

    private final OperatorInterface oi;

    private final PIDManager positionPID;

    ElevatorSubsystem elevator;

    final DoubleProperty humanMaxPowerGoingUp;
    final DoubleProperty humanMaxPowerGoingDown;
    // Creates a new TrapezoidProfile
    final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 1));
    double profileStartTime;
    Distance previousTarget;
    TrapezoidProfile.State initialState;
    TrapezoidProfile.State goalState;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi){
        super(elevator,pf,hvmFactory, 1, 0.2);
        this.elevator = elevator;
        previousTarget = elevator.getTargetValue();
        initialState = new TrapezoidProfile.State(elevator.getCurrentValue().in(Meters), elevator.getCurrentVelocity().in(MetersPerSecond));
        goalState = new TrapezoidProfile.State(elevator.getTargetValue().in(Meters), 0);

        this.oi = oi;
        positionPID = pidf.create(getPrefix() + "positionPID", 0.00, 0, 0.0);

        humanMaxPowerGoingUp = pf.createPersistentProperty("maxPowerGoingUp", 1);
        humanMaxPowerGoingDown = pf.createPersistentProperty("maxPowerGoingDown", -0.2);
    }

    @Override
    public void initialize() {
        log.info("initializing");
    }

    @Override
    protected void coastAction() {
        elevator.setPower(0);
    }

    @Override
    protected void initializeMachineControlAction() {
        profileStartTime = XTimer.getFPGATimestamp();
    }

    @Override
    protected void calibratedMachineControlAction() {
        // if the target has changed, reset the profile
        if (!previousTarget.equals(elevator.getTargetValue())) {
            previousTarget = elevator.getTargetValue();
            profileStartTime = XTimer.getFPGATimestamp();
            initialState = new TrapezoidProfile.State(elevator.getCurrentValue().in(Meters), elevator.getCurrentVelocity().in(MetersPerSecond));
            goalState = new TrapezoidProfile.State(elevator.getTargetValue().in(Meters), 0);
        }

        // determine setpoint from the profile
        var elapsedTime = XTimer.getFPGATimestamp() - profileStartTime;
        aKitLog.record("elapsedTime", elapsedTime);
        var setpoint = profile.calculate(elapsedTime, initialState, goalState);
        aKitLog.record("elevatorProfileTarget", setpoint.position);

        double power = positionPID.calculate(
                setpoint.position,
                elevator.getCurrentValue().in(Meters));
//        double power = (elevator.getTargetValue().in(Meters) - elevator.getCurrentValue().in(Meters)) * 0.5;
//        power = MathUtils.constrainDouble(power,-0.8, 1);
        elevator.setPower(power);

    }

    @Override
    protected void uncalibratedMachineControlAction() {
        //this is just a placeholder for now until we have something to calibrate
        humanControlAction();
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
                    oi.programmerGamepad.getLeftVector().getY(),
                    oi.getOperatorGamepadTypicalDeadband(),
                    (a) -> (a)),
                humanMaxPowerGoingDown.get(), humanMaxPowerGoingUp.get());
    }

    @Override
    protected double getHumanInputMagnitude() {
        return Math.abs(getHumanInput());
    }

}
