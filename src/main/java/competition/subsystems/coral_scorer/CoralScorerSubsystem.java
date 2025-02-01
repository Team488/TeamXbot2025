package competition.subsystems.coral_scorer;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.controls.sensors.XTimer;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class CoralScorerSubsystem extends BaseSubsystem {

    public enum CoralScorerState {
        INTAKING,
        SCORING,
        STOPPED
    }
    public final XCANMotorController motor;
    public final DoubleProperty intakePower;
    public final DoubleProperty scorePower;
    public final XDigitalInput coralSensor;
    public final ElectricalContract electricalContract;
    private CoralScorerState coralScorerState;
    private double lastScoredTime = -Double.MAX_VALUE;
    public final DoubleProperty waitTimeAfterScoring;

    @Inject
    public CoralScorerSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                                ElectricalContract electricalContract, PropertyFactory propertyFactory,
                                XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);
        if (electricalContract.isCoralCollectionMotorReady()) {
            this.motor = xcanMotorControllerFactory.create(electricalContract.getCoralCollectionMotor(),
                    getPrefix(), "CoralScorer");
            this.registerDataFrameRefreshable(motor);
        } else {
            this.motor = null;
        }

        if (electricalContract.isCoralSensorReady()) {
            this.coralSensor = xDigitalInputFactory.create(electricalContract.getCoralSensor(),
                    "CoralSensor");
            this.registerDataFrameRefreshable(coralSensor);
        } else {
            this.coralSensor = null;
        }

        this.coralScorerState = CoralScorerState.STOPPED;

        this.intakePower = propertyFactory.createPersistentProperty("intakePower", .1);
        this.scorePower = propertyFactory.createPersistentProperty("scorerPower", -.1);
        this.waitTimeAfterScoring = propertyFactory.createPersistentProperty("waitTimeAfterScoring", 0.5);

        this.electricalContract = electricalContract;
    }

    public void setCoralScorerMotorPower(double power) {
        if (electricalContract.isCoralCollectionMotorReady()) {
            this.motor.setPower(power);
        }
    }

    public void intake() {
        setCoralScorerMotorPower(intakePower.get());
        coralScorerState = CoralScorerState.INTAKING;
    }
    public void scorer() {
        setCoralScorerMotorPower(scorePower.get());
        if (coralScorerState != CoralScorerState.SCORING) {
            lastScoredTime = XTimer.getFPGATimestamp();
        }
        coralScorerState = CoralScorerState.SCORING;
    }
    public void stop() {
        setCoralScorerMotorPower(0);
        coralScorerState = CoralScorerState.STOPPED;
    }
    public boolean hasCoral() {
        if (electricalContract.isCoralSensorReady()) {
            return this.coralSensor.get();
        }
        return false;
    }
    public double getSecondsSinceScoringStarted() {
        if (coralScorerState != CoralScorerState.SCORING) {
            return 0;
        }
        return XTimer.getFPGATimestamp() - lastScoredTime;
    }
    public boolean confidentlyHasScoredCoral() {
        return getSecondsSinceScoringStarted() > waitTimeAfterScoring.get();
    }

    public void periodic() {
        aKitLog.record("coralPresent", this.hasCoral());
    }
}

