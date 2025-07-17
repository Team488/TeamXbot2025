package competition.subsystems.coral_scorer;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.wpilibj.Alert;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class CoralScorerSubsystem extends BaseSubsystem {
    public final XCANMotorController motor;
    public final XDigitalInput coralSensor;

    final Alert hasCoralAlert = new Alert("Confidently has coral", Alert.AlertType.kInfo);
    public final ElectricalContract electricalContract;

    DoubleProperty ejectMotorPower;
    DoubleProperty intakeMotorPower;

    @Inject
    public CoralScorerSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                                ElectricalContract electricalContract, PropertyFactory propertyFactory,
                                XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);

        ejectMotorPower = propertyFactory.createPersistentProperty("Eject Motor Speed", 1);
        intakeMotorPower = propertyFactory.createPersistentProperty("Intake Motor Speed", -1);

        if (electricalContract.isCoralCollectionMotorReady()) {
            this.motor = xcanMotorControllerFactory.create(electricalContract.getCoralCollectionMotor(),
                    getPrefix(), "CoralScorer");
            this.registerDataFrameRefreshable(motor);
        } else {
            this.motor = null;
        }

        if (electricalContract.isCoralScorerSensorReady()) {
            this.coralSensor = xDigitalInputFactory.create(electricalContract.getCoralScorerSensor(),
                    this.getPrefix());
            this.registerDataFrameRefreshable(coralSensor);
        } else {
            this.coralSensor = null;
        }

        this.electricalContract = electricalContract;
    }

    public void intake() {
        motor.setPower(intakeMotorPower.get());
    }

    public void eject() {
        motor.setPower(ejectMotorPower.get());
    }

    public void idle() {
        motor.setPower(0);
    }

    public boolean hasCoral() {
        // TODO: return true if the coral sensor is true
        return false;
    }

    public void periodic() {
        if (electricalContract.isCoralCollectionMotorReady()) {
            motor.periodic();
        }

        aKitLog.record("coralPresentFromSensor", hasCoral());
    }
}

