package competition.subsystems.coral_scorer;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class CoralScorerSubsystem extends BaseSubsystem {
    public XCANMotorController motor = null;
    public DoubleProperty intakePower;
    public DoubleProperty scorePower;
    public XDigitalInput coralSensor;
    public BooleanProperty isCoralReady;

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

        this.isCoralReady = propertyFactory.createPersistentProperty("isCoralReady", false);
    }

    public void intake() {
        if (motor != null){
            motor.setPower(intakePower.get());
        }
    }
    public void scorer() {
        if (motor != null){
            motor.setPower(scorePower.get());
        }
    }
    public void stop() {
        if (motor!=null){
            motor.setPower(0);
        }
    }
    public boolean hasCoral() {
        if (coralSensor != null) {
            isCoralReady.set(true);
        }
        return isCoralReady.get();
    }
}

