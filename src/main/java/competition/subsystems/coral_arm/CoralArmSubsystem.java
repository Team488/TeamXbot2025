package competition.subsystems.coral_arm;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.XCANMotorControllerPIDProperties;
import xbot.common.controls.sensors.XAbsoluteEncoder;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

@Singleton
public class CoralArmSubsystem extends BaseSetpointSubsystem<Angle> {

    public final XCANMotorController armMotor;
    public final XAbsoluteEncoder armAbsoluteEncoder;
    public final XDigitalInput lowSensor;
    Angle targetAngle = Degrees.of(0);
    ElectricalContract electricalContract;

    double rotationsAtZero = 0;
    boolean isCalibrated = true;

    public final DoubleProperty rotationsPerDegrees;
    public final Angle degreesPerRotations;

    public final DoubleProperty scoreAngle;
    public final DoubleProperty humanLoadAngle;
    public final DoubleProperty rangeOfMotionDegrees;
    public final DoubleProperty minArmPosition;
    public final DoubleProperty maxArmPosition;
    public final DoubleProperty powerWhenNotCalibrated;

    @Inject
    public CoralArmSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             ElectricalContract electricalContract, PropertyFactory propertyFactory,
                             XAbsoluteEncoder.XAbsoluteEncoderFactory xAbsoluteEncoderFactory,
                             XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);

        this.electricalContract = electricalContract;

        if (electricalContract.isCoralArmMotorReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getCoralArmPivotMotor(),
                    getPrefix(), "ArmPivotMotor", new XCANMotorControllerPIDProperties(0.2,0,0.2));

            this.armAbsoluteEncoder = xAbsoluteEncoderFactory.create(electricalContract.getCoralArmPivotAbsoluteEncoder(),
                    "ArmPivotAbsoluteEncoder");
            this.lowSensor = xDigitalInputFactory.create(electricalContract.getCoralArmPivotLowSensor(),
                    "ArmPivotLowSensor");
            this.registerDataFrameRefreshable(this.armMotor);
            this.registerDataFrameRefreshable(this.armAbsoluteEncoder);
            this.registerDataFrameRefreshable(this.lowSensor);
        } else {
            this.armMotor = null;
            this.armAbsoluteEncoder = null;
            this.lowSensor = null;
        }

        this.rotationsPerDegrees = propertyFactory.createPersistentProperty("Rotations Per Degrees", 10);
        this.degreesPerRotations = Degrees.of(rotationsPerDegrees.get() != 0 ? 1.0 / rotationsPerDegrees.get() : 0);
        if (rotationsPerDegrees.get() == 0) {log.warn("CANNOT DIVIDE BY 0!");}

        this.rangeOfMotionDegrees = propertyFactory.createPersistentProperty("Range of Motion in Degrees", 125);
        this.minArmPosition = propertyFactory.createPersistentProperty("Min AbsEncoder Position in Degrees", 90);
        this.maxArmPosition = propertyFactory.createPersistentProperty("Max AbsEncoder Position in Degrees", 108);
        this.scoreAngle = propertyFactory.createPersistentProperty("Scoring Angle in Degrees", 125);
        this.humanLoadAngle = propertyFactory.createPersistentProperty("Human Loading Angle in Degrees", 0);
        this.powerWhenNotCalibrated = propertyFactory.createPersistentProperty("Power When Not Calibrated", 0.05);

    }




    @Override
    public Angle getCurrentValue() {
        Angle currentAngle = Degrees.of(0);
        if (electricalContract.isCoralArmMotorReady()) {
            currentAngle = Degrees.of(
                            (calibratedPosition().in(Rotations)) * degreesPerRotations.in(Degrees));
        }
        return currentAngle;
    }

    private Angle calibratedPosition() {
        return getMotorPosition().minus(Rotations.of(rotationsAtZero));
    }

    private Angle getMotorPosition() {
        if (electricalContract.isCoralArmMotorReady()) {
            return this.armMotor.getPosition();
        }
        return Rotations.of(0);
    }

    @Override
    public Angle getTargetValue() {
        return targetAngle;
    }

    public AngularVelocity getCurrentVelocity() {
        return DegreesPerSecond.of(armMotor.getVelocity().in(RotationsPerSecond) * degreesPerRotations.in(Degrees));
    }

    @Override
    public void setTargetValue(Angle value) {
        targetAngle = value;
    }

    public void setTargetAngle(Landmarks.CoralLevel value) {
        switch (value) {
            case ONE:
            case TWO:
            case THREE:
            case FOUR:
                setTargetValue(Degrees.of(scoreAngle.get()));
                break;
            case COLLECTING:
                setTargetValue(Degrees.of(humanLoadAngle.get()));
                break;
            default:
                setTargetValue(Degrees.of(humanLoadAngle.get()));
                break;
        }
    }

    @Override
    public void setPower(double power) {
        if (electricalContract.isCoralArmMotorReady()) {
            if (getCurrentValue().in(Degrees) <= humanLoadAngle.get()) {
                log.info("human load" + humanLoadAngle.get());
                power = MathUtils.constrainDouble(power, 0, 1);
            }
            if (getCurrentValue().in(Degrees) > scoreAngle.get()) {
                log.info(scoreAngle.get());
                power = MathUtils.constrainDouble(power, -1, 0);
            }
            if (!isCalibrated()) {
                power = MathUtils.constrainDouble(power, -powerWhenNotCalibrated.get(), powerWhenNotCalibrated.get());
            }

            this.armMotor.setPower(power);
        }
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    public void setCalibrated(boolean calibrated){
        isCalibrated = calibrated;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Angle target1, Angle target2) {
        return target1.isEquivalent(target2);
    }

    public Angle getArmAngle() {
        if (electricalContract.isCoralArmPivotAbsoluteEncoderReady() && electricalContract.isCoralArmPivotLowSensorReady()) {
            return getArmAngle(minArmPosition.get() / 360, maxArmPosition.get() / 360,
                    armAbsoluteEncoder.getAbsolutePosition(), lowSensor.get(), rangeOfMotionDegrees.get());
        }
        return Angle.ofBaseUnits(0, Degrees);
    }

    public static Angle getArmAngle(double minPosition, double maxPosition,
                                    Angle absEncoderAngle, boolean sensorHit, double rangeOfMotionDegrees) {
        double armPosition = 0;
        double absEncoderPosition = absEncoderAngle.in(Degrees) / 360;
        double tolerance;

        // Check for the special case when maxPosition < minPosition
        if (maxPosition < minPosition) {
            tolerance = 1 - minPosition + maxPosition;
            if (sensorHit) {
                if (absEncoderPosition > minPosition) {
                    armPosition = absEncoderPosition - minPosition;
                }
                else {
                    armPosition = 1 - minPosition + absEncoderPosition;
                }
            }
            else if (absEncoderPosition > minPosition) {
                armPosition = 1 - tolerance + (absEncoderPosition - minPosition);
            }
            else if (absEncoderPosition < maxPosition) {
                armPosition = 1 - maxPosition + absEncoderPosition;
            }
            else {
                armPosition = 1 - minPosition + absEncoderPosition;
            }
        }
        // Use separate math when maxPosition > minPosition
        else {
            if (absEncoderPosition < minPosition || absEncoderPosition > maxPosition) {
                if (absEncoderPosition < minPosition) {
                    armPosition = (absEncoderPosition + 1 - maxPosition);
                } else {
                    armPosition = (absEncoderPosition - minPosition);
                }
            } else if (absEncoderPosition > minPosition && absEncoderPosition < maxPosition) {
                if (sensorHit) {
                    armPosition = (absEncoderPosition - minPosition);
                } else {
                    armPosition = 1 - (maxPosition - absEncoderPosition);
                }
            }
        }

        // convert from [0,1] position to arm angle in degrees
        return Degrees.of(armPosition * rangeOfMotionDegrees);

    }

    @Override
    public void periodic() {
        if (electricalContract.isCoralArmMotorReady()) {
            armMotor.periodic();
        }

        aKitLog.record("Target Angle", this.getTargetValue().in(Degrees));
        aKitLog.record("Current Angle", this.getCurrentValue().in(Degrees));
        aKitLog.record("isCalibrated", this.isCalibrated());
        aKitLog.record("Current Angle using AbsEncoder", this.getArmAngle().in(Degrees));
        if(electricalContract.isAlgaeArmBottomSensorReady()) {
            aKitLog.record("lowSensor Status", lowSensor.get());
        }
    }
  
    public boolean getIsTargetAngleScoring() {
        return Degrees.of(scoreAngle.get()).isNear(targetAngle, Degrees.of(10));
    }
}