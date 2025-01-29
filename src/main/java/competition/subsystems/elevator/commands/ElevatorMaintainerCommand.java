package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
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

    ElevatorSubsystem elevator;

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
        positionPID = pidf.create(getPrefix() + "positionPID", 0.5, 0.01, 0);

        humanMaxPowerGoingUp = pf.createPersistentProperty("maxPowerGoingUp", 1);
        humanMaxPowerGoingDown = pf.createPersistentProperty("maxPowerGoingDown", -0.05);
    }

    @Override
    public void initialize() {
        log.info("initializing");
        //initializeMachineControlAction();

    }

    @Override
    protected void coastAction() {
        elevator.setPower(0);
    }

    @Override
    protected void calibratedMachineControlAction() {
//        if(isMaintainerAtGoal()){
//            elevator.setPower(0.0);
//        }
//        else {
            double power = positionPID.calculate(
                    elevator.getTargetValue().in(Meters),
                    elevator.getCurrentValue().in(Meters));
            elevator.setPower(power);
      //  }

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

        return Math.abs(current.in(Inches)/target.in(Inches));
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
