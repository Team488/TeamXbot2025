package competition.subsystems.arm_pivot;

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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

@Singleton
public class ArmPivotSubsystem extends BaseSetpointSubsystem<Angle> {
    public final XCANMotorController armMotor;
    public final XAbsoluteEncoder armAbsoluteEncoder;
    public final XDigitalInput lowSensor;
    Angle targetAngle;
    ElectricalContract electricalContract;
    DoubleProperty degreesPerRotations;
    double rotationsAtZero;
    boolean isCalibrated = false;
    final DoubleProperty rangeOfMotionDegrees;
    final DoubleProperty minArmPosition;
    final DoubleProperty maxArmPosition;

    @Inject
    public ArmPivotSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             ElectricalContract electricalContract, PropertyFactory propertyFactory,
                             XAbsoluteEncoder.XAbsoluteEncoderFactory xAbsoluteEncoderFactory,
                             XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {

        propertyFactory.setPrefix(this);
        this.electricalContract = electricalContract;

        if (electricalContract.isArmPivotReady()) {
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
    }




    @Override
    public Angle getCurrentValue() {
        double currentAngle = (this.armMotor.getPosition().in(Rotations) - rotationsAtZero) * degreesPerRotations.get();
        return Degrees.of(currentAngle);
    }

    @Override
    public Angle getTargetValue() {
        return targetAngle;
    }

    @Override
    public void setTargetValue(Angle value) {
        targetAngle = value;
    }

    @Override
    public void setPower(double power) {
        if (electricalContract.isArmPivotReady()) {
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
        // if armAbsoluteEncoder is null, return 0
        if (armAbsoluteEncoder.getAbsolutePosition() == null) {
            return Angle.ofBaseUnits(0, Degrees);
        }
        return getArmAngle(minArmPosition.get(), minArmPosition.get(),
                armAbsoluteEncoder.getAbsolutePosition(), lowSensor.get(), rangeOfMotionDegrees.get());
    }

    public static Angle getArmAngle(double minPosition, double maxPosition,
                                    Angle absEncoderAngle, boolean sensorHit, double rangeOfMotionDegrees) {
        double armPosition = 0;
        double absEncoderPosition = absEncoderAngle.in(Degrees) / 360;

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
        // convert from [0,1] position to arm angle in degrees
        return Degrees.of(armPosition * rangeOfMotionDegrees);
    }
}