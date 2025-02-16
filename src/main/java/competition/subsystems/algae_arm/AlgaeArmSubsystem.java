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
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

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

    XDigitalInput dio0;
    XDigitalInput dio1;
    XDigitalInput dio2;
    XDigitalInput dio3;
    XDigitalInput dio4;
    XDigitalInput dio5;
    XDigitalInput dio6;
    XDigitalInput dio7;
    XDigitalInput dio8;
    XDigitalInput dio9;

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
        } else {
            this.bottomSensor = null;
        }
        this.degreesPerRotation = propertyFactory.createPersistentProperty("DegreesPerRotation", 1);

        this.rangeOfMotionInDegrees = propertyFactory.createPersistentProperty("RangeOfMotionInDegrees", 160.0);
        this.groundCollectionDegrees = propertyFactory.createPersistentProperty("GroundCollectionDegrees", 45.0);
        this.reefLowBottomToTopSweepStart = propertyFactory.createPersistentProperty("ReefLowBottomToTopSweepStart", 90.0);
        this.reefLowBottomToTopSweepEnd = propertyFactory.createPersistentProperty("ReefLowBottomToTopSweepEnd", 150.0);
        this.reefLowTopToBottomSweepStart = propertyFactory.createPersistentProperty("ReefLowTopToBottomSweepStart", 150.0);
        this.reefLowTopToBottomSweepEnd = propertyFactory.createPersistentProperty("ReefLowTopToBottomSweepEnd", 90.0);
        this.reefHighSweepStart = propertyFactory.createPersistentProperty("ReefHighSweepStart", 110.0);
        this.reefHighSweepEnd = propertyFactory.createPersistentProperty("ReefHighSweepEnd", 150.0);

        dio0 = xDigitalInputFactory.create(new DeviceInfo("Dio0", 0), this.getPrefix());
        dio1 = xDigitalInputFactory.create(new DeviceInfo("Dio1", 1), this.getPrefix());
        dio2 = xDigitalInputFactory.create(new DeviceInfo("Dio2", 2), this.getPrefix());
        dio3 = xDigitalInputFactory.create(new DeviceInfo("Dio3", 3), this.getPrefix());
        dio4 = xDigitalInputFactory.create(new DeviceInfo("Dio4", 4), this.getPrefix());
        dio5 = xDigitalInputFactory.create(new DeviceInfo("Dio5", 5), this.getPrefix());
        dio6 = xDigitalInputFactory.create(new DeviceInfo("Dio6", 6), this.getPrefix());
        dio7 = xDigitalInputFactory.create(new DeviceInfo("Dio7", 7), this.getPrefix());
        dio8 = xDigitalInputFactory.create(new DeviceInfo("Dio8", 8), this.getPrefix());
        dio9 = xDigitalInputFactory.create(new DeviceInfo("Dio9", 9), this.getPrefix());

        registerDataFrameRefreshable(dio0);
        registerDataFrameRefreshable(dio1);
        registerDataFrameRefreshable(dio2);
        registerDataFrameRefreshable(dio3);
        registerDataFrameRefreshable(dio4);
        registerDataFrameRefreshable(dio5);
        registerDataFrameRefreshable(dio6);
        registerDataFrameRefreshable(dio7);
        registerDataFrameRefreshable(dio8);
        registerDataFrameRefreshable(dio9);
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
        return getMotorVelocity();
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

        aKitLog.record("dio0", dio0.get());
        aKitLog.record("dio1", dio1.get());
        aKitLog.record("dio2", dio2.get());
        aKitLog.record("dio3", dio3.get());
        aKitLog.record("dio4", dio4.get());
        aKitLog.record("dio5", dio5.get());
        aKitLog.record("dio6", dio6.get());
        aKitLog.record("dio7", dio7.get());
        aKitLog.record("dio8", dio8.get());
        aKitLog.record("dio9", dio9.get());
    }
}




