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
    boolean isCalibrated = false;
    private final DoubleProperty degreesPerRotations;

    public final DoubleProperty scoreAngle;
    public final DoubleProperty humanLoadAngle;
    public final DoubleProperty rangeOfMotionDegrees;
    public final DoubleProperty minArmPosition;
    public final DoubleProperty maxArmPosition;
    public final DoubleProperty powerWhenNotCalibrated;

    private final DoubleProperty minRotations;
    private final DoubleProperty maxRotations;

    @Inject
    public CoralArmSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             ElectricalContract electricalContract, PropertyFactory propertyFactory,
                             XAbsoluteEncoder.XAbsoluteEncoderFactory xAbsoluteEncoderFactory,
                             XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);

        this.electricalContract = electricalContract;

        if (electricalContract.isCoralArmMotorReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getCoralArmPivotMotor(),
                    getPrefix(), "ArmPivotMotor", new XCANMotorControllerPIDProperties(
                            2,
                            0,
                            0,
                            0,
                            0,
                            0.4,
                            -0.25
            ));
            this.registerDataFrameRefreshable(this.armMotor);
        } else {
            this.armMotor = null;
        }

        if (electricalContract.isCoralArmPivotAbsoluteEncoderReady()) {
            this.armAbsoluteEncoder = xAbsoluteEncoderFactory.create(electricalContract.getCoralArmPivotAbsoluteEncoder(),
                    this.getPrefix());
            this.registerDataFrameRefreshable(this.armAbsoluteEncoder);
        } else {
            this.armAbsoluteEncoder = null;
        }

        if (electricalContract.isCoralArmPivotLowSensorReady()) {
            this.lowSensor = xDigitalInputFactory.create(electricalContract.getCoralArmPivotLowSensor(),
                    this.getPrefix());
            this.registerDataFrameRefreshable(this.lowSensor);
        } else {
            this.lowSensor = null;
        }

        this.degreesPerRotations = propertyFactory.createPersistentProperty("Degrees Per Rotations", 6.94444);

        this.rangeOfMotionDegrees = propertyFactory.createPersistentProperty("Range of Motion in Degrees", 125);
        this.minArmPosition = propertyFactory.createPersistentProperty("Min AbsEncoder Position in Degrees", 90);
        this.maxArmPosition = propertyFactory.createPersistentProperty("Max AbsEncoder Position in Degrees", 108);
        this.scoreAngle = propertyFactory.createPersistentProperty("Scoring Angle in rotations", 23);
        this.humanLoadAngle = propertyFactory.createPersistentProperty("Human Loading Angle in rotations", 5);
        this.powerWhenNotCalibrated = propertyFactory.createPersistentProperty("Power When Not Calibrated", 0.05);

        this.minRotations = propertyFactory.createPersistentProperty("Min Rotations", 0);
        this.maxRotations = propertyFactory.createPersistentProperty("Max Rotations", 23);
    }

    @Override
    public Angle getCurrentValue() {
        // Temporarily using Rotations everywhere until we have a way of better
        // measuring the physical location of the arm.
        return getCalibratedPosition();
    }

    private Angle getAbsoluteAngle() {
        Angle currentAngle = Degrees.of(0);
        if (electricalContract.isCoralArmMotorReady()) {
            currentAngle = Degrees.of(
                    (getCalibratedPosition().in(Rotations)) * degreesPerRotations.get());
        }
        return currentAngle;
    }

    private Angle getMotorRotations() {
        if (electricalContract.isCoralArmMotorReady()) {
            return getMotorPosition();
        }
        return Rotations.of(0);
    }

    private Angle getCalibratedPosition() {
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
        return getMotorVelocity();
    }

    private AngularVelocity getAbsoluteVelocity() {
        return DegreesPerSecond.of(armMotor.getVelocity().in(RotationsPerSecond) * degreesPerRotations.get());
    }

    private AngularVelocity getMotorVelocity() {
        return armMotor.getVelocity();
    }

    public double getDegreesPerRotation() {
        return degreesPerRotations.get();
    }

    public double getRotationsPerDegree() {
        // if the degreesPerRotations is 0, return 0 to avoid division by zero
        return degreesPerRotations.get() == 0 ? 0 : 1 / degreesPerRotations.get();
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
                setTargetValue(Rotations.of(scoreAngle.get()));
                break;
            case COLLECTING:
            default:
                setTargetValue(Rotations.of(humanLoadAngle.get()));
                break;
        }
    }

    @Override
    public void setPower(double power) {
        if (electricalContract.isCoralArmMotorReady()) {
            if (getCurrentValue().in(Rotations) < minRotations.get()) {
                power = MathUtils.constrainDouble(power, 0, 1);
            }
            if (getCurrentValue().in(Rotations) > maxRotations.get()) {
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

    public void forceCalibratedHere() {
        if (electricalContract.isCoralArmMotorReady()) {
            rotationsAtZero = getMotorPosition().in(Rotations);
        }
        isCalibrated = true;
    }

    public void setPositionalGoalIncludingOffset(Angle setpoint) {
        armMotor.setPositionTarget(
                Rotations.of(setpoint.in(Rotations) + rotationsAtZero),
                XCANMotorController.MotorPidMode.Voltage);
    }

    @Override
    public void periodic() {
        if (electricalContract.isCoralArmMotorReady()) {
            armMotor.periodic();
        }

        aKitLog.record("Target Angle", this.getTargetValue().in(Rotations));
        aKitLog.record("Current Angle", this.getCurrentValue().in(Rotations));
        aKitLog.record("isCalibrated", this.isCalibrated());
        if (electricalContract.isCoralArmPivotAbsoluteEncoderReady()) {
            aKitLog.record("Current Angle using AbsEncoder", this.getArmAngle().in(Degrees));
        }
        if(electricalContract.isAlgaeArmBottomSensorReady()) {
            aKitLog.record("lowSensor Status", lowSensor.get());
        }
    }
  
    public boolean getIsTargetAngleScoring() {
        return Degrees.of(scoreAngle.get()).isNear(targetAngle, Rotations.of(0.25));
    }
}