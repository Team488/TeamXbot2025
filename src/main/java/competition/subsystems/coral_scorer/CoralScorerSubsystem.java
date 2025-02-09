package competition.subsystems.coral_scorer;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.oracle.contracts.CoralCollectionInfoSource;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.controls.sensors.XTimer;
import xbot.common.logic.TimeStableValidator;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static competition.subsystems.coral_scorer.CoralScorerSubsystem.CoralScorerState.INTAKING;
import static competition.subsystems.coral_scorer.CoralScorerSubsystem.CoralScorerState.SCORING;
import static competition.subsystems.coral_scorer.CoralScorerSubsystem.CoralScorerState.STOPPED;

@Singleton
public class CoralScorerSubsystem extends BaseSubsystem implements CoralCollectionInfoSource {

    public enum CoralScorerState {
        INTAKING,
        SCORING,
        STOPPED
    }

    public final XCANMotorController motor;
    public final DoubleProperty intakePower;
    public final DoubleProperty hasCoralIntakePower;
    public final DoubleProperty scorePower;
    public final XDigitalInput coralSensor;
    public final ElectricalContract electricalContract;
    private CoralScorerState coralScorerState;
    private double lastScoredTime = -Double.MAX_VALUE;
    public final DoubleProperty waitTimeAfterScoring;
    public final DoubleProperty waitTimeAfterCollection;

    private final TimeStableValidator hasCoralValidator;

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

        this.coralScorerState = STOPPED;

        this.intakePower = propertyFactory.createPersistentProperty("intakePower", 0.1);
        this.hasCoralIntakePower = propertyFactory.createPersistentProperty("hasCoralIntakePower", 0.01);
        this.scorePower = propertyFactory.createPersistentProperty("scorerPower", -0.1);
        this.waitTimeAfterScoring = propertyFactory.createPersistentProperty("waitTimeAfterScoring", 0.5);
        this.waitTimeAfterCollection = propertyFactory.createPersistentProperty("waitTimeAfterCollection", 0.1);

        hasCoralValidator = new TimeStableValidator(waitTimeAfterCollection::get);

        this.electricalContract = electricalContract;
    }

    public void setCoralScorerMotorPower(double power) {
        if (electricalContract.isCoralCollectionMotorReady()) {
            this.motor.setPower(power);
        }
    }

    public void setPowerForState(CoralScorerState state) {
        switch (state) {
            case INTAKING:
                intake();
                break;
            case SCORING:
                scorer();
                break;
            case STOPPED:
            default:
                stop();
                break;
        }
    }

    public void intake() {
        if (confidentlyHasCoral()) {
            setCoralScorerMotorPower(hasCoralIntakePower.get());
        } else {
            setCoralScorerMotorPower(intakePower.get());
        }
        coralScorerState = INTAKING;
    }
    public void scorer() {
        setCoralScorerMotorPower(scorePower.get());
        if (coralScorerState != SCORING) {
            lastScoredTime = XTimer.getFPGATimestamp();
        }
        coralScorerState = SCORING;
    }
    public void stop() {
        setCoralScorerMotorPower(0);
        coralScorerState = STOPPED;
    }
    public boolean hasCoral() {
        if (electricalContract.isCoralSensorReady()) {
            return this.coralSensor.get();
        }
        return false;
    }
    public double getSecondsSinceScoringStarted() {
        if (coralScorerState != SCORING) {
            return 0;
        }
        return XTimer.getFPGATimestamp() - lastScoredTime;
    }
    public boolean confidentlyHasScoredCoral() {
        return (getSecondsSinceScoringStarted() > waitTimeAfterScoring.get() && coralScorerState == SCORING);
    }

    @Override
    public boolean confidentlyHasCoral() {
        return hasCoralValidator.peekStable();
    }

    public CoralScorerState getCoralScorerState() {
        return coralScorerState;
    }

    public void periodic() {
        if (electricalContract.isCoralCollectionMotorReady()) {
            motor.periodic();
        }

        boolean hasCoral = hasCoral();
        hasCoralValidator.checkStable(this.hasCoral());

        aKitLog.record("coralPresent", hasCoral());
        aKitLog.record("coralPresentStable", hasCoralValidator.peekStable());
        aKitLog.record("CoralConfidentlyScored", confidentlyHasScoredCoral());
    }
}

