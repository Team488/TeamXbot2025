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
    private final XAbsoluteEncoder armAbsoluteEncoder;
    private final XDigitalInput lowSensor;
    Angle targetAngle;
    ElectricalContract electricalContract;
    DoubleProperty degreesPerRotations;
    double rotationsAtZero;
    boolean isCalibrated = false;
    double positionToDegrees = 125.0;
    DoubleProperty minArmPosition;
    DoubleProperty maxArmPosition;

    @Inject
    public ArmPivotSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             ElectricalContract electricalContract, PropertyFactory propertyFactory,
                             XAbsoluteEncoder.XAbsoluteEncoderFactory xAbsoluteEncoderFactory,
                             XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {

        propertyFactory.setPrefix(this);
        this.electricalContract = electricalContract;

        // should be electricalContract.isArmPivotSubsystemReady
        if (electricalContract.isArmPivotReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getArmPivotMotor(),
                    getPrefix(), "ArmPivotMotor");
            this.armAbsoluteEncoder = xAbsoluteEncoderFactory.create(electricalContract.getArmPivotAbsoluteEncoder(),
                    "ArmPivotAbsoluteEncoder");
            this.lowSensor = xDigitalInputFactory.create(electricalContract.getArmPivotLowSensor(),
                    "ArmPivotLowSensor");
        } else {
            this.armMotor = null;
            this.armAbsoluteEncoder = null;
            this.lowSensor = null;
        }

        this.degreesPerRotations = propertyFactory.createPersistentProperty("Degrees Per Rotations", 1);
        this.minArmPosition = propertyFactory.createPersistentProperty("Min AbsEncoder Position", 0.25);
        this.maxArmPosition = propertyFactory.createPersistentProperty("Max AbsEncoder Position", 0.28);
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

    // method should receive absolute encoder position and return the arm's angle in degrees
    public Angle getArmAngle() {
        double absoluteEncoderPosition = armAbsoluteEncoder.getAbsolutePosition().in(Degrees) / 360.0;
        double armPosition = 0;

        if (absoluteEncoderPosition < minArmPosition.get() || absoluteEncoderPosition > maxArmPosition.get()) {
            if (absoluteEncoderPosition < minArmPosition.get()) {
                armPosition = (absoluteEncoderPosition + 1 - maxArmPosition.get());
            }
            else {
                armPosition = (absoluteEncoderPosition - minArmPosition.get());
            }
        }
        else if (absoluteEncoderPosition > minArmPosition.get() && absoluteEncoderPosition < maxArmPosition.get()){
            if (lowSensor.get()) {
                armPosition = (absoluteEncoderPosition - minArmPosition.get());
            }
            else {
                armPosition = 1 - (maxArmPosition.get() - absoluteEncoderPosition);
            }
        }
        // convert from [0,1] position to arm angle in degrees.
        return Degrees.of(armPosition * positionToDegrees);
    }

    public static Angle getArmAngleTest(double minPosition, double maxPosition,
                                        double absEncoderPosition, boolean sensorHit, double positionToDegrees) {
        double absoluteEncoderPosition = absEncoderPosition / 360.0;
        double armPosition = 0;

        if (absoluteEncoderPosition < minPosition || absoluteEncoderPosition > maxPosition) {
            if (absoluteEncoderPosition < minPosition) {
                armPosition = (absoluteEncoderPosition + 1 - maxPosition);
            }
            else {
                armPosition = (absoluteEncoderPosition - minPosition);
            }
        }
        else if (absoluteEncoderPosition > minPosition && absoluteEncoderPosition < maxPosition){
            if (sensorHit) {
                armPosition = (absoluteEncoderPosition - minPosition);
            }
            else {
                armPosition = 1 - (maxPosition - absoluteEncoderPosition);
            }
        }
        // convert from [0,1] position to arm angle in degrees.
        return Degrees.of(armPosition * positionToDegrees);
    }
}