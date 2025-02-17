package competition.subsystems.algae_arm;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

@Singleton
public class AlgaeArmSubsystem extends BaseSetpointSubsystem<Angle> {
    public final XCANMotorController armMotor;
    Angle targetAngle = Degree.of(0);
    final ElectricalContract electricalContract;
    double rotationsAtZero;
    boolean isCalibrated = false;
    public final XDigitalInput bottomSensor;

    final DoubleProperty degreesPerRotation;
    final DoubleProperty rangeOfMotionInDegrees;
    final DoubleProperty groundCollectionDegrees;
    final DoubleProperty reefLowBottomToTopSweepStart;
    final DoubleProperty reefLowBottomToTopSweepEnd;
    final DoubleProperty reefLowTopToBottomSweepStart;
    final DoubleProperty reefLowTopToBottomSweepEnd;
    final DoubleProperty reefHighSweepStart;
    final DoubleProperty reefHighSweepEnd;

    final Alert isNotCalibratedAlert = new Alert("AlgaeArm: not calibrated", Alert.AlertType.kWarning);

    public enum AlgaeArmPositions {
        FullyRetracted,
        GroundCollection,
        ReefAlgaeLow,
        ReefAlgaeHigh
    }

    @Inject
    public AlgaeArmSubsystem(ElectricalContract electricalContract,
                             XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             PropertyFactory propertyFactory,
                             XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);
        this.electricalContract = electricalContract;
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getAlgaeArmPivotMotor(),
                    getPrefix(), "AlgaeArmPivotMotor");
            this.registerDataFrameRefreshable(this.armMotor);
        } else {
            this.armMotor = null;
        }

        if (electricalContract.isAlgaeArmBottomSensorReady()) {
            this.bottomSensor = xDigitalInputFactory.create(electricalContract.getAlgaeArmBottomSensor(), this.getPrefix());
            registerDataFrameRefreshable(this.bottomSensor);
        } else {
            this.bottomSensor = null;
        }
        this.degreesPerRotation = propertyFactory.createPersistentProperty("DegreesPerRotation", 13.523);

        this.rangeOfMotionInDegrees = propertyFactory.createPersistentProperty("RangeOfMotionInDegrees", 160.0);
        this.groundCollectionDegrees = propertyFactory.createPersistentProperty("GroundCollectionDegrees", 45.0);
        this.reefLowBottomToTopSweepStart = propertyFactory.createPersistentProperty("ReefLowBottomToTopSweepStart", 90.0);
        this.reefLowBottomToTopSweepEnd = propertyFactory.createPersistentProperty("ReefLowBottomToTopSweepEnd", 150.0);
        this.reefLowTopToBottomSweepStart = propertyFactory.createPersistentProperty("ReefLowTopToBottomSweepStart", 150.0);
        this.reefLowTopToBottomSweepEnd = propertyFactory.createPersistentProperty("ReefLowTopToBottomSweepEnd", 90.0);
        this.reefHighSweepStart = propertyFactory.createPersistentProperty("ReefHighSweepStart", 110.0);
        this.reefHighSweepEnd = propertyFactory.createPersistentProperty("ReefHighSweepEnd", 150.0);
    }

    @Override
    public Angle getCurrentValue() {
        double currentAngle = 0;
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
            currentAngle = getCalibratedPosition().in(Rotations) * degreesPerRotation.get();
        }
        return Degrees.of(currentAngle);
    }

    private Angle getCalibratedPosition() {
        return getMotorPosition().minus(Rotations.of(rotationsAtZero));
    }

    private Angle getMotorPosition() {
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
            return this.armMotor.getPosition();
        }
        return Rotations.of(0);
    }

    public AngularVelocity getCurrentVelocity() {
        return DegreesPerSecond.of(
                getMotorVelocity().in(RotationsPerSecond) * degreesPerRotation.get()
        );
    }

    private AngularVelocity getMotorVelocity() {
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
            return this.armMotor.getVelocity();
        }
        return RadiansPerSecond.zero();
    }

    @Override
    public Angle getTargetValue() {
        return targetAngle;
    }

    @Override
    public void setTargetValue(Angle value) {
        targetAngle = value;
    }

    public void setTargetValue(AlgaeArmPositions position) {
        switch (position) {
            case GroundCollection -> setTargetValue(Degrees.of(groundCollectionDegrees.get()));
            case ReefAlgaeLow -> setTargetValue(Degrees.of(reefLowBottomToTopSweepStart.get()));
            case ReefAlgaeHigh -> setTargetValue(Degrees.of(reefHighSweepStart.get()));
            default -> setTargetValue(Degrees.of(0));
        }
    }

    public void setPositionalGoalIncludingOffset(Angle setpoint) {
        armMotor.setPositionTarget(
                Rotations.of(setpoint.in(Degrees) / degreesPerRotation.get() + rotationsAtZero),
                XCANMotorController.MotorPidMode.Voltage);
    }

    public void forceCalibratedHere() {
        setCalibrated();
    }

    public void setCalibrated() {
        isCalibrated = true;
        rotationsAtZero = getMotorPosition().in(Rotations);
    }

    @Override
    public void setPower(double power) {
        if (electricalContract.isAlgaeArmPivotMotorReady()) {

            if (isCalibrated) {
                double currentAngle = getCurrentValue().in(Degrees);
                if (currentAngle < 0) {
                    power = MathUtils.constrainDouble(power, 0, 1);
                }
                if (currentAngle > rangeOfMotionInDegrees.get()) {
                    power = MathUtils.constrainDouble(power, -1, 0);
                }
            }

            if (isTouchingBottom()) {
                power = MathUtils.constrainDouble(power, 0, 1);
            }

            this.armMotor.setPower(power);
        }

    }

    public boolean isTouchingBottom(){
        if (electricalContract.isAlgaeArmBottomSensorReady()){
            return this.bottomSensor.get();
        }
        return false;
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Angle targetAngle1, Angle targetAngle2) {
        return targetAngle1.isEquivalent(targetAngle2);
    }

    @Override
    public void periodic() {
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
            armMotor.periodic();
        }
        isNotCalibratedAlert.set(!isCalibrated());
        aKitLog.record("Target Angle", this.getTargetValue().in(Degrees));
        aKitLog.record("Current Angle", this.getCurrentValue().in(Degrees));
        aKitLog.record("isCalibrated", this.isCalibrated());
        aKitLog.record("Current Velocity", this.getCurrentVelocity().in(DegreesPerSecond));
        aKitLog.record("Touching Bottom", this.isTouchingBottom());
    }
}




