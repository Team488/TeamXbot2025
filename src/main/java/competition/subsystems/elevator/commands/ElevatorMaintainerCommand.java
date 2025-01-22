package competition.subsystems.elevator.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.command.BaseSubsystem;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.PIDManager;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class ElevatorMaintainerCommand extends BaseMaintainerCommand<Distance> {

    public enum MaintainerMode{
        Calibrating,
        GaveUp,
        Calibrated,
    }

    ElevatorSubsystem elevator;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi){
        super(elevator,pf,hvmFactory, 1, 0.2);

    }

    @Override
    public void initialize() {
        log.info("initializing");
    }

    @Override
    public void execute(){

    }

    @Override
    protected void coastAction() {

    }

    @Override
    protected void calibratedMachineControlAction() {

    }

    @Override
    protected double getErrorMagnitude() {
        return 0;
    }

    @Override
    protected double getHumanInput() {
        return 0;
    }

    @Override
    protected double getHumanInputMagnitude() {
        return 0;
    }

}
