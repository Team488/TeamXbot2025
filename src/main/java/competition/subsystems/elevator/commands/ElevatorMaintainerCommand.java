package competition.subsystems.elevator.commands;

import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import javax.inject.Inject;
import javax.inject.Provider;

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

    final TrapezoidProfileManager profileManager;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, Provider<PropertyFactory> pfProvider,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi){
        super(elevator, pfProvider.get(),hvmFactory, 1, 0.2);
        var pf = pfProvider.get();
        pf.setPrefix(this);
        this.elevator = elevator;
        profileManager = new TrapezoidProfileManager(getPrefix() + "trapezoidMotion", pfProvider.get(), 1, 1, elevator.getCurrentValue().in(Meters));

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
    protected void calibratedMachineControlAction() {

        profileManager.setTargetPosition(
            elevator.getTargetValue().in(Meters),
            elevator.getCurrentValue().in(Meters),
            elevator.getCurrentVelocity().in(MetersPerSecond)
        );
        var setpoint = profileManager.getRecommendedPositionForTime();

        // it's helpful to log this to know where the robot is actually trying to get to in the moment
        aKitLog.record("elevatorProfileTarget", setpoint);

        double power = positionPID.calculate(
                setpoint,
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
