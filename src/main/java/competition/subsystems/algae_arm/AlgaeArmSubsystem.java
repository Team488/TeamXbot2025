package competition.subsystems.algae_arm;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;


@Singleton
public class AlgaeArmSubsystem extends BaseSetpointSubsystem<Angle> {
    public final Optional<XCANMotorController> maybeMotor;
    public final Optional<XDigitalInput> maybeBottomSensor;
    final DoubleProperty degreesPerRotation;
    
    double rotationsAtZero;
    boolean isCalibrated = false;
    Angle targetAngle = Degree.of(0);

    @Inject
    public AlgaeArmSubsystem(ElectricalContract electricalContract,
                             XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             PropertyFactory propertyFactory,
                             XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
            var motor = xcanMotorControllerFactory.create(electricalContract.getAlgaeArmPivotMotor(),
                    getPrefix(), "AlgaeArmPivotMotor");
            this.registerDataFrameRefreshable(motor);
            this.maybeMotor = Optional.of(motor);
        } else{
            this.maybeMotor = Optional.empty();
        }

        if (electricalContract.isAlgaeArmBottomSensorReady()){
            var sensor = xDigitalInputFactory.create(electricalContract.getAlgaeArmBottomSensor(), this.getPrefix());
            this.registerDataFrameRefreshable(sensor);
            this.maybeBottomSensor = Optional.of(sensor);
        } else{
            this.maybeBottomSensor = Optional.empty();
        }
        this.degreesPerRotation = propertyFactory.createPersistentProperty("DegreesPerRotation", 1);
    }

    @Override
    public Angle getCurrentValue() {
        return this.maybeMotor
                .map((motor) -> {
                    return Degrees.of(
                            (motor.getPosition().in(Rotations) - rotationsAtZero) * degreesPerRotation.get());
                })
                .orElse(Degrees.zero());
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
        this.maybeMotor.ifPresent((motor) -> motor.setPower(power));
    }

    public boolean isTouchingBottom(){
        return this.maybeBottomSensor.map((sensor) -> sensor.get()).orElse(false);
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Angle targetAngle1, Angle targetAngle2) {
        return targetAngle1.isEquivalent(targetAngle2);
    }
        


}




