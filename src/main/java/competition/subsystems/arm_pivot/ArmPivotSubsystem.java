package competition.subsystems.arm_pivot;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ArmPivotSubsystem extends BaseSetpointSubsystem {
    public final XCANMotorController armMotor;

    @Inject
    public ArmPivotSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             ElectricalContract electricalContract, PropertyFactory propertyFactory) {

        propertyFactory.setPrefix(this);
        if (electricalContract.isArmPivotMotorReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getArmPivotMotor(),
                    getPrefix(), "ArmPivotMotor");
        } else {
            this.armMotor = null;
        }
    }

    @Override
    public Object getCurrentValue() {
        return null;
    }

    @Override
    public Object getTargetValue() {
        return null;
    }

    @Override
    public void setTargetValue(Object value) {

    }

    @Override
    public void setPower(Object power) {

    }

    @Override
    public boolean isCalibrated() {
        return false;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Object target1, Object target2) {
        return false;
    }
}