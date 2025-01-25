package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.command.BaseSubsystem;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;

import javax.inject.Inject;

public class ElevatorMaintainerCommand extends BaseMaintainerCommand<Distance> {

    public enum MaintainerMode{
        Calibrating,
        GaveUp,
        Calibrated,
    }

    private final OperatorInterface oi;

    ElevatorSubsystem elevator;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi){
        super(elevator,pf,hvmFactory, 1, 0.2);

        this.oi = oi;

    }

    @Override
    public void initialize() {
        log.info("initializing");

    }

    @Override
    public void execute(){
        maintain();
    }

    @Override
    protected void coastAction() {
        elevator.setPower(0);
    }

    @Override
    protected void calibratedMachineControlAction() {

    }

    @Override
    protected void uncalibratedMachineControlAction() {
        //this is just a placeholder for now until we have something to calibrate
        humanControlAction();
    }

    @Override
    protected void humanControlAction() {
        elevator.setPower(getHumanInput());
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
        return MathUtils.deadband(
                oi.programmerGamepad.getLeftVector().getY(),
                0.15);
    }

    @Override
    protected double getHumanInputMagnitude() {
        return Math.abs(getHumanInput());
    }

}
