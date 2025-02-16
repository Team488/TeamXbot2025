package competition.subsystems.coral_scorer;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.oracle.contracts.CoralCollectionInfoSource;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.controls.sensors.XTimer;
import xbot.common.logic.TimeStableValidator;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static competition.subsystems.coral_scorer.CoralScorerSubsystem.CoralScorerState.SCORING;
import static competition.subsystems.coral_scorer.CoralScorerSubsystem.CoralScorerState.STOPPED;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
    final Alert hasCoralAlert = new Alert("Confidently has coral", Alert.AlertType.kInfo);
    public final ElectricalContract electricalContract;
    private CoralScorerState coralScorerState;
    private double lastScoredTime = -Double.MAX_VALUE;
    public final DoubleProperty waitTimeAfterScoring;
    public final DoubleProperty waitTimeAfterCollection;

    private final TimeStableValidator hasCoralValidator;

    private final DoubleProperty intakeFreeSpeedRPSProperty;

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
                    this.getPrefix());
            this.registerDataFrameRefreshable(coralSensor);
        } else {
            this.coralSensor = null;
        }

        this.coralScorerState = STOPPED;

        this.intakePower = propertyFactory.createPersistentProperty("intakePower", 0.1);
        this.hasCoralIntakePower = propertyFactory.createPersistentProperty("hasCoralIntakePower", 0.05);
        this.scorePower = propertyFactory.createPersistentProperty("scorerPower", -0.1);
        this.waitTimeAfterScoring = propertyFactory.createPersistentProperty("waitTimeAfterScoring", 0.5);
        this.waitTimeAfterCollection = propertyFactory.createPersistentProperty("waitTimeAfterCollection", 0.1);

        this.intakeFreeSpeedRPSProperty = propertyFactory.createPersistentProperty("intakeFreeSpeedRPS", 3);

        hasCoralValidator = new TimeStableValidator(waitTimeAfterCollection::get);

        this.electricalContract = electricalContract;
    }

    public void setCoralScorerState(CoralScorerState state) {
        if (coralScorerState != SCORING && state == SCORING) {
            lastScoredTime = XTimer.getFPGATimestamp();;
        }
        coralScorerState = state;
    }

    private void setCoralScorerMotorPower(double power) {
        if (electricalContract.isCoralCollectionMotorReady()) {
            this.motor.setPower(power);
        }
    }

    private void setPowerForState(CoralScorerState state) {
        switch (state) {
            case INTAKING:
                intake();
                break;
            case SCORING:
                score();
                break;
            case STOPPED:
            default:
                stop();
                break;
        }
    }

    private void intake() {
        if (confidentlyHasCoral()) {
            setCoralScorerMotorPower(hasCoralIntakePower.get());
        } else {
            setCoralScorerMotorPower(intakePower.get());
        }
    }

    private void score() {
        setCoralScorerMotorPower(scorePower.get());
        if (coralScorerState != SCORING) {
            lastScoredTime = XTimer.getFPGATimestamp();
    }
    }

    private void stop() {
        setCoralScorerMotorPower(0);
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


    private boolean coralLikelyJammed() {
        return coralScorerState == CoralScorerState.INTAKING
                && Math.abs(motor.getVelocity().in(RotationsPerSecond)) < intakeFreeSpeedRPSProperty.get();

    }

    public AngularVelocity getMotorVelocity() {
        if(electricalContract.isCoralCollectionMotorReady()) {
            return motor.getVelocity();
        }

        return RotationsPerSecond.zero();
    }

    public void periodic() {
        if (electricalContract.isCoralCollectionMotorReady()) {
            motor.periodic();
        }

        switch (coralScorerState) {
            case INTAKING:
                intake();
                break;
            case SCORING:
                score();
                break;
            case STOPPED:
            default:
                stop();
                break;
        }

        boolean coralLikelyJammed = coralLikelyJammed();
        hasCoralValidator.checkStable(this.hasCoral() || coralLikelyJammed);

        aKitLog.record("coralPresentFromSensor", hasCoral());
        hasCoralAlert.set(confidentlyHasCoral());
        aKitLog.record("coralPresentFromJamming", coralLikelyJammed);
        aKitLog.record("coralPresentStable", hasCoralValidator.peekStable());
        aKitLog.record("CoralConfidentlyScored", confidentlyHasScoredCoral());
        aKitLog.record("IntakeRPS", getMotorVelocity().in(RotationsPerSecond));
        aKitLog.record("coralScorerState", coralScorerState);
        aKitLog.record("secondsSinceScoring", getSecondsSinceScoringStarted());
    }
}

