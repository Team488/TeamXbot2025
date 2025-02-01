package competition.subsystems.coral_arm_pivot;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XAbsoluteEncoder;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import java.util.Objects;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

@Singleton
public class CoralArmPivotSubsystem extends BaseSetpointSubsystem<Angle> {

    public enum ArmGoals {
        Score,
        HumanLoad
    }

    public final DoubleProperty scoreAngle;
    public final DoubleProperty humanLoadAngle;

    public final XCANMotorController armMotor;
    public final XAbsoluteEncoder armAbsoluteEncoder;
    public final XDigitalInput lowSensor;
    Angle targetAngle = Degrees.of(0);
    ElectricalContract electricalContract;
    DoubleProperty degreesPerRotations;
    double rotationsAtZero;
    boolean isCalibrated = false;
    final DoubleProperty rangeOfMotionDegrees;
    final DoubleProperty minArmPosition;
    final DoubleProperty maxArmPosition;

    @Inject
    public CoralArmPivotSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                                  ElectricalContract electricalContract, PropertyFactory propertyFactory,
                                  XAbsoluteEncoder.XAbsoluteEncoderFactory xAbsoluteEncoderFactory,
                                  XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);

        this.electricalContract = electricalContract;

        if (electricalContract.isArmPivotMotorReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getArmPivotMotor(),
                    getPrefix(), "ArmPivotMotor");
            this.armAbsoluteEncoder = xAbsoluteEncoderFactory.create(electricalContract.getArmPivotAbsoluteEncoder(),
                    "ArmPivotAbsoluteEncoder");
            this.lowSensor = xDigitalInputFactory.create(electricalContract.getArmPivotLowSensor(),
                    "ArmPivotLowSensor");
            this.registerDataFrameRefreshable(this.armMotor);
            this.registerDataFrameRefreshable(this.armAbsoluteEncoder);
            this.registerDataFrameRefreshable(this.lowSensor);
        } else {
            this.armMotor = null;
            this.armAbsoluteEncoder = null;
            this.lowSensor = null;
        }

        this.degreesPerRotations = propertyFactory.createPersistentProperty("Degrees Per Rotations", 1);

        this.rangeOfMotionDegrees = propertyFactory.createPersistentProperty("Range of Motion in Degrees", 125);
        this.minArmPosition = propertyFactory.createPersistentProperty("Min AbsEncoder Position in Degrees", 90);
        this.maxArmPosition = propertyFactory.createPersistentProperty("Max AbsEncoder Position in Degrees", 100);
        this.scoreAngle = propertyFactory.createPersistentProperty("Scoring Angle", -125);
        this.humanLoadAngle = propertyFactory.createPersistentProperty("Human Loading Angle", 0);

    }




    @Override
    public Angle getCurrentValue() {
        double currentAngle = getMotorPositionFromZeroOffset().in(Rotations) * degreesPerRotations.get();
        return Degrees.of(currentAngle);
    }

    private Angle getMotorPositionFromZeroOffset() {
        return getMotorPosition().minus(Rotations.of(rotationsAtZero));
    }

    private Angle getMotorPosition() {
        if (electricalContract.isArmPivotMotorReady()) {
            return this.armMotor.getPosition();
        }
        return Rotations.of(0);
    }

    @Override
    public Angle getTargetValue() {
        return targetAngle;
    }

    @Override
    public void setTargetValue(Angle value) {
        targetAngle = value;
    }

    public void setTargetAngle(ArmGoals value) {
        if (Objects.requireNonNull(value) == ArmGoals.Score) {
            setTargetValue(Degrees.of(scoreAngle.get()));
        } else {
            setTargetValue(Degrees.of(humanLoadAngle.get()));
        }
    }

    @Override
    public void setPower(double power) {
        if (getMotorPositionFromZeroOffset().in(Rotations) < 0 && power < 0) {
                power = 0;
        }
        if (electricalContract.isArmPivotMotorReady()) {
            this.armMotor.setPower(power);
        }
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Angle target1, Angle target2) {
        return target1.isEquivalent(target2);
    }

    public Angle getArmAngle() {
        if (electricalContract.isArmPivotAbsoluteEncoderReady() && electricalContract.isArmPivotLowSensorReady()) {
            return getArmAngle(minArmPosition.get(), maxArmPosition.get(),
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
    public boolean getIsTargetAngleScoring() {
        return targetAngle == Degrees.of(scoreAngle.get());
    }
}