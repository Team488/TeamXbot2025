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

import static competition.subsystems.coral_scorer.CoralScorerSubsystem.CoralScorerState.SCORING_CORAL;
import static competition.subsystems.coral_scorer.CoralScorerSubsystem.CoralScorerState.STOPPED;
import static edu.wpi.first.units.Units.RotationsPerSecond;

@Singleton
public class CoralScorerSubsystem extends BaseSubsystem implements CoralCollectionInfoSource {

    public enum CoralScorerState {
        INTAKING_CORAL,
        SCORING_CORAL,
        INTAKING_ALGAE,
        SCORING_ALGAE,
        STOPPED
    }

    public final XCANMotorController motor;
    public final DoubleProperty intakeCoralPower;
    public final DoubleProperty hasCoralIntakePower;
    public final DoubleProperty scoreCoralPower;
    public final DoubleProperty intakeAlgaePower;
    public final DoubleProperty scoreAlgaePower;
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

        if (electricalContract.isCoralScorerSensorReady()) {
            this.coralSensor = xDigitalInputFactory.create(electricalContract.getCoralScorerSensor(),
                    this.getPrefix());
            this.registerDataFrameRefreshable(coralSensor);
        } else {
            this.coralSensor = null;
        }

        this.coralScorerState = STOPPED;

        this.intakeCoralPower = propertyFactory.createPersistentProperty("intakeCoralPower", 0.3);
        this.hasCoralIntakePower = propertyFactory.createPersistentProperty("hasCoralIntakePower", 0.05);
        this.scoreCoralPower = propertyFactory.createPersistentProperty("scorerPower", -0.8);
        this.intakeAlgaePower = propertyFactory.createPersistentProperty("intakeAlgaePower", 0.5);
        this.scoreAlgaePower = propertyFactory.createPersistentProperty("scoreAlgaePower", 1);
        this.waitTimeAfterScoring = propertyFactory.createPersistentProperty("waitTimeAfterScoring", 0.2);
        this.waitTimeAfterCollection = propertyFactory.createPersistentProperty("waitTimeAfterCollection", 0.1);

        this.intakeFreeSpeedRPSProperty = propertyFactory.createPersistentProperty("intakeFreeSpeedRPS", 3);

        hasCoralValidator = new TimeStableValidator(waitTimeAfterCollection::get);

        this.electricalContract = electricalContract;
    }

    public void setCoralScorerState(CoralScorerState state) {
        if (coralScorerState != SCORING_CORAL && state == SCORING_CORAL) {
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
            case INTAKING_CORAL:
                intakeCoral();
                break;
            case SCORING_CORAL:
                scoreCoral();
                break;
            case STOPPED:
            default:
                stop();
                break;
        }
    }

    private void intakeCoral() {
        if (confidentlyHasCoral()) {
            setCoralScorerMotorPower(hasCoralIntakePower.get());
        } else {
            setCoralScorerMotorPower(intakeCoralPower.get());
        }
    }

    private void scoreCoral() {
        setCoralScorerMotorPower(scoreCoralPower.get());
        if (coralScorerState != SCORING_CORAL) {
            lastScoredTime = XTimer.getFPGATimestamp();
        }
    }

    private void intakeAlgae() {
        setCoralScorerMotorPower(intakeAlgaePower.get());
        if (coralScorerState != SCORING_CORAL) {
            lastScoredTime = XTimer.getFPGATimestamp();
        }
    }

    private void scoreAlgae() {
        setCoralScorerMotorPower(scoreCoralPower.get());
    }

    private void stop() {
        setCoralScorerMotorPower(0);
    }

    public boolean hasCoral() {
        if (electricalContract.isCoralScorerSensorReady()) {
            return this.coralSensor.get();
        }
        return false;
    }
    public double getSecondsSinceScoringStarted() {
        if (coralScorerState != SCORING_CORAL) {
            return 0;
        }
        return XTimer.getFPGATimestamp() - lastScoredTime;
    }
    public boolean confidentlyHasScoredCoral() {
        return (getSecondsSinceScoringStarted() > waitTimeAfterScoring.get() && coralScorerState == SCORING_CORAL);
    }

    @Override
    public boolean confidentlyHasCoral() {
        return hasCoralValidator.peekStable();
    }

    public CoralScorerState getCoralScorerState() {
        return coralScorerState;
    }


    private boolean coralLikelyJammed() {
        return coralScorerState == CoralScorerState.INTAKING_CORAL
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
            case INTAKING_CORAL:
                intakeCoral();
                break;
            case SCORING_CORAL:
                scoreCoral();
                break;
            case INTAKING_ALGAE:
                intakeAlgae();
                break;
            case SCORING_ALGAE:
                scoreAlgae();
                break;
            case STOPPED:
            default:
                stop();
                break;
        }

        boolean coralLikelyJammed = coralLikelyJammed();
        hasCoralValidator.checkStable(this.hasCoral() /*|| coralLikelyJammed*/);

        aKitLog.record("coralPresentFromSensor", hasCoral());
        hasCoralAlert.set(confidentlyHasCoral());
        aKitLog.record("coralPresentFromJamming", coralLikelyJammed);
        aKitLog.record("coralPresentStable", hasCoralValidator.peekStable());
        aKitLog.record("CoralConfidentlyScored", confidentlyHasScoredCoral());
        aKitLog.record("IntakeRPS", getMotorVelocity().in(RotationsPerSecond));
        aKitLog.record("coralScorerState", coralScorerState);
    }
}

