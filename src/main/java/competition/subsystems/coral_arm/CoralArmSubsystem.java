package competition.subsystems.coral_arm;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Alert;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.XCANMotorControllerPIDProperties;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.controls.sensors.XDutyCycleEncoder;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.Property.PropertyLevel;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Degrees;

@Singleton
public class CoralArmSubsystem extends BaseSetpointSubsystem<Angle> {

    public final XCANMotorController armMotor;
    public final XDutyCycleEncoder armAbsoluteEncoder;
    public final XDigitalInput lowSensor;
    public final MutAngle targetAngle = Degrees.mutable(0);
    ElectricalContract electricalContract;

    double periodicTickCounter;
    double rotationsAtZero = 0;
    boolean isCalibrated = false;
    final Alert isNotCalibratedAlert = new Alert("CoralArm: not calibrated", Alert.AlertType.kWarning);

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
    }

    @Override
    public Angle getCurrentValue() {
        double currentAngle = 0;
        // TODO: return the current angle of the arm
        return Degrees.of(currentAngle);
    }

    @Override
    public Angle getTargetValue() {
        // TODO: return the target angle
        return Degrees.zero();
    }

    @Override
    public void setTargetValue(Angle value) {
        // TODO: remember the target value
    }

    @Override
    public void setPower(double power) {
        // TODO: set the power to the motor
    }

    @Override
    public boolean isCalibrated() {
        // TODO: return whether the arm is calibrated
        return false;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Angle target1, Angle target2) {
        return target1.isEquivalent(target2);
    }

    @Override
    public void periodic() {
        if (electricalContract.isCoralArmMotorReady()) {
            armMotor.periodic();
        }


        aKitLog.record("Target Angle", this.getTargetValue().in(Degrees));
        aKitLog.record("Current Angle", this.getCurrentValue().in(Degrees));
        aKitLog.record("isCalibrated", this.isCalibrated());
        aKitLog.record("IsAtMaintainerGoal", this.isMaintainerAtGoal());
        isNotCalibratedAlert.set(!isCalibrated());
    }
}