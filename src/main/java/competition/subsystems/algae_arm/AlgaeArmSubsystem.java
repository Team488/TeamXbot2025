package competition.subsystems.algae_arm;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Angle;
import xbot.common.advantage.AKitLogger;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static competition.simulation.elevator.ElevatorSimConstants.rotationsAtZero;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

@Singleton
public class AlgaeArmSubsystem extends BaseSetpointSubsystem<Angle> {
    public final XCANMotorController armMotor;
    Angle targetAngle = Degree.of(0);
    final ElectricalContract electricalContract;
    final DoubleProperty degreesPerRotation;
    double rotationsAtZero;
    final boolean isCalibrated = false;
    public final XDigitalInput bottomSensor;

    @Inject
    public AlgaeArmSubsystem(ElectricalContract electricalContract,
                             XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory, PropertyFactory propertyFactory, XDigitalInput.XDigitalInputFactory xDigitalInputFactory) {
        propertyFactory.setPrefix(this);
        this.electricalContract = electricalContract;
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getArmPivotMotor(),
                    getPrefix(), "AlgaeArmPivotMotor");
            this.registerDataFrameRefreshable(this.armMotor);
        } else{
            this.armMotor=null;
        }

        if (electricalContract.isAlgaeArmBottomSensorReady()){
            this.bottomSensor= xDigitalInputFactory.create(electricalContract.getElevatorBottomSensor(), "ElevatorBottomSensor");
        } else{
            this.bottomSensor=null;
        }
        this.degreesPerRotation = propertyFactory.createPersistentProperty("DegreesPerRotation", 1);
    }





    @Override
    public Angle getCurrentValue() {
        double currentAngle = 0;
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
            currentAngle = (this.armMotor.getPosition().in(Rotations) - rotationsAtZero) * degreesPerRotation.get();
        }
        //double currentAngle = getMotorPositionFromZeroOffset().in(Rotations) * degreesPerRotations.get();
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
        if (electricalContract.isAlgaeArmPivotMotorReady()) {
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
        


}




