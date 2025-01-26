package competition.subsystems.coral_scorer;

import competition.electrical_contract.ElectricalContract;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class CoralScorerSubsystem extends BaseSubsystem implements DataFrameRefreshable {
    public final XCANMotorController motor;
    public final DoubleProperty intakePower;
    public final DoubleProperty scorePower;
    public final XDigitalInput coralSensor;
    public final ElectricalContract electricalContract;

    @Inject
    public CoralScorerSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                                ElectricalContract electricalContract, PropertyFactory propertyFactory,
                                XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);
        if (electricalContract.isCoralCollectionMotorReady()) {
            this.motor = xcanMotorControllerFactory.create(electricalContract.getCoralCollectionMotor(),
                    getPrefix(), "CoralScorer");
        } else {
            this.motor = null;
        }

        if (electricalContract.isCoralSensorReady()) {
            this.coralSensor = xDigitalInputFactory.create(electricalContract.getCoralSensor(),
                    "CoralSensor");
        } else {
            this.coralSensor = null;
        }

        this.intakePower = propertyFactory.createPersistentProperty("intakePower", .1);
        this.scorePower = propertyFactory.createPersistentProperty("scorerPower", -.1);

        this.electricalContract = electricalContract;
    }

    public void setCoralScorerMotorPower(double power) {
        if (electricalContract.isCoralCollectionMotorReady()) {
            this.motor.setPower(power);
        }
    }

    public void intake() {
        setCoralScorerMotorPower(intakePower.get());
    }
    public void scorer() {
        setCoralScorerMotorPower(scorePower.get());
    }
    public void stop() {
        setCoralScorerMotorPower(0);
    }
    public boolean hasCoral() {
        if (electricalContract.isCoralSensorReady()) {
            return this.coralSensor.get();
        }
        return false;
    }

    public void periodic() {
        aKitLog.record("coralPresent", this.hasCoral());
    }
    @Override
    public void refreshDataFrame() {
        if(electricalContract.isCoralSensorReady()) {
            coralSensor.refreshDataFrame();
        }
    }
}

