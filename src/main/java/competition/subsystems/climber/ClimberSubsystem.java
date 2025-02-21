package competition.subsystems.climber;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ClimberSubsystem extends BaseSubsystem {

    public final XCANMotorController climberMotor;
    public final DoubleProperty climberPower;
    public final ElectricalContract contract;

    @Inject
    public ClimberSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                            ElectricalContract contract, PropertyFactory pf) {
        pf.setPrefix(this);
        if (contract.isClimberMotorReady()) {
            this.climberMotor = xcanMotorControllerFactory.create(contract.getClimberMotor(),
                    getPrefix(), "ClimberMotor");
            this.registerDataFrameRefreshable(climberMotor);
        } else {
            this.climberMotor = null;
        }

        this.contract = contract;

        this.climberPower = pf.createPersistentProperty("climberPower", 0.1);
    }

    public void setPower(double power) {
        if (contract.isClimberMotorReady()) {
            this.climberMotor.setPower(power);
        }
    }

    public void stop() {
        if (contract.isClimberMotorReady()) {
            this.climberMotor.setPower(0);
        }
    }
}
