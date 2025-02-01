package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.logic.TimeStableValidator;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Meters;

import javax.inject.Inject;

public class ElevatorMaintainerCommand extends BaseMaintainerCommand<Distance> {

    public enum MaintainerMode{
        Calibrating,
        GaveUp,
        Calibrated,
    }

    private final OperatorInterface oi;

    private final PIDManager positionPID;

    private final ElevatorSubsystem elevator;

    boolean startedCalibration = false;
    boolean givenUpOnCalibration = false;
    double calibrationStartTime = 0;
    double giveUpCalibratingTime;
    final double calibrationMaxDuration = 5;
    TimeStableValidator calibrationValidator;

    final DoubleProperty humanMaxPowerGoingUp;
    final DoubleProperty humanMaxPowerGoingDown;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi){
        super(elevator,pf,hvmFactory, 1, 0.2);
        this.elevator = elevator;

        this.oi = oi;
        positionPID = pidf.create(getPrefix() + "positionPID", 0.1, 0, 0.5);

        this.humanMaxPowerGoingUp = pf.createPersistentProperty("maxPowerGoingUp", 1);
        this.humanMaxPowerGoingDown = pf.createPersistentProperty("maxPowerGoingDown", -0.2);
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
        double power;
//        if (elevator.isMaintainerAtGoal()){
//            power = 0;
//        }
 //       else {
            power = positionPID.calculate(
                    elevator.getTargetValue().in(Meters),
                    elevator.getCurrentValue().in(Meters));
        //}
        elevator.setPower(power);

    }

    @Override
    protected void uncalibratedMachineControlAction() {
        aKitLog.record("Started on calibration", startedCalibration);
        aKitLog.record("Given up on calibration", givenUpOnCalibration);

        if (!startedCalibration){
            calibrationStartTime = XTimer.getFPGATimestamp();
            startedCalibration = true;
        }

        if (calibrationStartTime + calibrationMaxDuration < XTimer.getFPGATimestamp()){
            givenUpOnCalibration = true;
        }

        if (!givenUpOnCalibration){
            elevator.setPower(elevator.maxPowerWhenUncalibrated.get());

            if (elevator.isTouchingBottom()){
                elevator.markElevatorAsCalibratedAgainstLowerLimit();
                elevator.setTargetValue(elevator.getCurrentValue());
            }
        }
        else{
            humanControlAction();
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
