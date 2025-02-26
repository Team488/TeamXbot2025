package competition.subsystems.coral_arm;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.XCANMotorControllerPIDProperties;
import xbot.common.controls.sensors.XAbsoluteEncoder;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.controls.sensors.XDutyCycleEncoder;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

@Singleton
public class CoralArmSubsystem extends BaseSetpointSubsystem<Angle> {

    public final XCANMotorController armMotor;
    public final XDutyCycleEncoder armAbsoluteEncoder;
    public final XDigitalInput lowSensor;
    Angle targetAngle = Degrees.of(0);
    ElectricalContract electricalContract;

    double periodicTickCounter;
    double rotationsAtZero = 0;
    boolean isCalibrated = false;
    final Alert isNotCalibratedAlert = new Alert("CoralArm: not calibrated", Alert.AlertType.kWarning);

    private final DoubleProperty degreesPerRotations;
    public final DoubleProperty scoreAngleDegrees;
    public final DoubleProperty humanLoadAngleDegrees;
    public final DoubleProperty rangeOfMotionDegrees;
    public final DoubleProperty powerWhenNotCalibrated;

    public Landmarks.CoralLevel targetCoralLevel;

    @Inject
    public CoralArmSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             ElectricalContract electricalContract, PropertyFactory propertyFactory,
                             XDutyCycleEncoder.XDutyCycleEncoderFactory xDutyCycleEncoderFactory,
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
            this.armAbsoluteEncoder = xDutyCycleEncoderFactory.create(electricalContract.getCoralArmPivotAbsoluteEncoder());
            this.registerDataFrameRefreshable(this.armAbsoluteEncoder);
        } else {
            this.armAbsoluteEncoder = null;
        }

        if (electricalContract.isCoralArmLowSensorReady()) {
            this.lowSensor = xDigitalInputFactory.create(electricalContract.getCoralArmLowSensor(),
                    this.getPrefix());
            this.registerDataFrameRefreshable(this.lowSensor);
        } else {
            this.lowSensor = null;
        }

        this.degreesPerRotations = propertyFactory.createPersistentProperty("Degrees Per Rotations", 5.790);
        this.rangeOfMotionDegrees = propertyFactory.createPersistentProperty("Range of Motion in Degrees", 125);
        this.scoreAngleDegrees = propertyFactory.createPersistentProperty("Scoring Angle in Degrees", 125);
        this.humanLoadAngleDegrees = propertyFactory.createPersistentProperty("Human Loading Angle in Degrees", 0);
        this.powerWhenNotCalibrated = propertyFactory.createPersistentProperty("Power When Not Calibrated", 0.25);
    }

    @Override
    public Angle getCurrentValue() {
        double currentAngle = 0;
        if (electricalContract.isCoralArmMotorReady()) {
            currentAngle = getCalibratedPosition().in(Rotations) * degreesPerRotations.get();
        }
        return Degrees.of(currentAngle);
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
        return DegreesPerSecond.of(getMotorVelocity().in(RotationsPerSecond) * degreesPerRotations.get());
    }

    private AngularVelocity getAbsoluteVelocity() {
        return DegreesPerSecond.of(armMotor.getVelocity().in(RotationsPerSecond) * degreesPerRotations.get());
    }

    private AngularVelocity getMotorVelocity() {
        if (electricalContract.isCoralArmMotorReady()) {
            return this.armMotor.getVelocity();
        }
        return RadiansPerSecond.zero();
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
                setTargetValue(Degrees.of(scoreAngleDegrees.get()));
                break;
            case COLLECTING:
            default:
                setTargetValue(Degrees.of(humanLoadAngleDegrees.get()));
                break;
        }
    }

    public boolean isTouchingBottom() {
        if (electricalContract.isCoralArmLowSensorReady()) {
            return this.lowSensor.get();
        }
        return false;
    }

    @Override
    public void setPower(double power) {
        if (electricalContract.isCoralArmMotorReady()) {
            if (isCalibrated) {
                double currentLocationInDegrees = getCurrentValue().in(Degrees);

                if (currentLocationInDegrees > rangeOfMotionDegrees.get()) {
                    power = MathUtils.constrainDouble(power, -1, 0);
                }

                if (currentLocationInDegrees < 0) {
                    power = MathUtils.constrainDouble(power, 0, 1);
                }
            } else {
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
        if (electricalContract.isCoralArmPivotAbsoluteEncoderReady() && electricalContract.isCoralArmLowSensorReady()) {
            return getArmAngle(0, rangeOfMotionDegrees.get() / 360,
                    Degrees.of(armAbsoluteEncoder.getAbsoluteDegrees()), lowSensor.get(), rangeOfMotionDegrees.get());
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
                Rotations.of(setpoint.in(Degrees) / degreesPerRotations.get() + rotationsAtZero),
                XCANMotorController.MotorPidMode.Voltage);
    }

    @Override
    public void periodic() {
        if (electricalContract.isCoralArmMotorReady()) {
            armMotor.periodic();
        }

        if (!isCalibrated) {
            periodicTickCounter++;
            if (this.isTouchingBottom() && periodicTickCounter >= 20) {
                forceCalibratedHere();
                setTargetValue(getCurrentValue());
            }
        }

        aKitLog.record("Target Angle", this.getTargetValue().in(Degrees));
        aKitLog.record("Current Angle", this.getCurrentValue().in(Degrees));
        aKitLog.record("isCalibrated", this.isCalibrated());
        isNotCalibratedAlert.set(!isCalibrated());
        if (electricalContract.isCoralArmPivotAbsoluteEncoderReady()) {
            aKitLog.record("Current Angle using AbsEncoder", this.getArmAngle().in(Degrees));
        }
        if(electricalContract.isAlgaeArmBottomSensorReady()) {
            aKitLog.record("lowSensor Status", lowSensor.get());
        }
        aKitLog.record("Is Sensor Active", this.isTouchingBottom());
    }
  
    public boolean getIsTargetAngleScoring() {
        return Degrees.of(scoreAngleDegrees.get()).isNear(targetAngle, Degrees.of(0.25));
    }

    
    public Angle getHumanLoadAngle() {
        return Degrees.of(this.humanLoadAngleDegrees.get());
    }

    public Command createSetTargetCoralLevelCommand(Landmarks.CoralLevel coralLevel) {
        return Commands.runOnce(() -> setTargetCoralLevel(coralLevel));
    }
    public void setTargetCoralLevel(Landmarks.CoralLevel coralLevel) {
        this.targetCoralLevel = coralLevel;
    }

    public Landmarks.CoralLevel getTargetCoralLevel() {
        return this.targetCoralLevel;
    }
}