package competition.subsystems.elevator.commands;

import competition.electrical_contract.ElectricalContract;
import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.CalibrationDecider;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.PIDManager;
import xbot.common.properties.PropertyFactory;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import javax.inject.Inject;

public class ElevatorMaintainerCommand extends BaseMaintainerCommand<Distance> {

    private final OperatorInterface oi;

    private final ElevatorSubsystem elevator;

    final ElectricalContract contract;

    @Inject
    public ElevatorMaintainerCommand(ElevatorSubsystem elevator, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory,
                                     CalibrationDecider.CalibrationDeciderFactory calibrationDeciderFactory,
                                     TrapezoidProfileManager.Factory trapezoidProfileManagerFactory,
                                     PIDManager.PIDManagerFactory pidf,
                                     OperatorInterface oi,
                                     ElectricalContract contract){
        super(elevator, pf, hvmFactory, Inches.of(1.5).in(Meters), 0.1);
        pf.setPrefix(this);
        this.elevator = elevator;
        
        this.oi = oi;
        this.contract = contract;
    }

    @Override
    protected void coastAction() {
        // TODO: during coast, we need to apply a little power so the elevator doesn't drop immediately before machine control kicks in
    }

    @Override
    protected void calibratedMachineControlAction() {
        // TODO: based on the elevator's current and target value, tell the elevator motor what to do
    }

    @Override
    protected void humanControlAction() {
        // TODO: read human input from the operator gamepad and set the elevator motor power accordingly
    }

    @Override
    protected double getHumanInput() {
        // TODO: return the human input from the operator gamepad, scaled to a range of -1 to 1
        return 0;
    }

    @Override
    protected double getHumanInputMagnitude() {
        return Math.abs(getHumanInput());
    }

    @Override
    protected double getErrorMagnitude() {
        // TODO: return the absolute value of the difference between the elevator's current value and target value
        // this is used to determine if the elevator is close enough to the target value
        return 0;
    }



}
