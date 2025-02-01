package competition.subsystems.algae_arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degrees;

public class AlgaeArmMaintainerCommand extends BaseMaintainerCommand<Angle> {
    final DoubleProperty humanMinPower;
    final DoubleProperty humanMaxPower;
    final AlgaeArmSubsystem algaeArmSubsystem;
    final OperatorInterface oi;

    @Inject
    public AlgaeArmMaintainerCommand(AlgaeArmSubsystem algaeArmSubsystem, PropertyFactory pf,
                                     HumanVsMachineDecider.HumanVsMachineDeciderFactory hvm,
                                     PIDManager.PIDManagerFactory pid,
                                     OperatorInterface oi) {
        super(algaeArmSubsystem, pf, hvm, 1, 1);
        this.algaeArmSubsystem = algaeArmSubsystem;
        this.oi = oi;
        pf.setPrefix(this);
        pf.setDefaultLevel(Property.PropertyLevel.Important);
        humanMaxPower = pf.createPersistentProperty("HumanMaxPowerProperty", 1);
        humanMinPower = pf.createPersistentProperty("HumanMinPowerProperty", -.1);
    }


    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    protected void coastAction() {
    }

    @Override
    protected void calibratedMachineControlAction() {

    }

    @Override
    protected double getErrorMagnitude() {
        Angle errorMagnitude = algaeArmSubsystem.getCurrentValue().minus(algaeArmSubsystem.getTargetValue());


        return errorMagnitude.in(Degrees);

    }

    @Override
    protected double getHumanInput() {
        return MathUtils.constrainDouble(
                MathUtils.deadband(
                        oi.algaeAndSysIdGamepad.getRightStickY(),
                        oi.getOperatorGamepadTypicalDeadband(),
                        (a) -> (a)),
                humanMinPower.get(), humanMaxPower.get());
    }

    @Override
    protected double getHumanInputMagnitude() {
        return getHumanInput();
    }

}







