package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XLaserCAN;
import xbot.common.properties.DoubleProperty;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

@Singleton
public class ElevatorSubsystem extends BaseSetpointSubsystem<Distance> {

    public enum ElevatorGoals{
        ScoreL1,
        ScoreL2,
        ScoreL3,
        ScoreL4,
        HumanLoad,
        ReturnToBase,
    }

    final ElectricalContract contract;

    // elevator starts uncalibrated because it could be in the middle of it's range and we have no idea where that is
    private boolean isCalibrated;
    //TODO: Add a calibration routine

    public Distance elevatorTargetHeight;
    final Distance distanceFromTargetHeight;

    final DoubleProperty metersPerRotation;

    public XCANMotorController masterMotor;

    //important heights
    public final DoubleProperty l1Height;
    public final DoubleProperty l2Height;
    public final DoubleProperty l3Height;
    public final DoubleProperty l4Height;
    public final DoubleProperty humanLoadHeight;
    public final DoubleProperty baseHeight;

    public final XDigitalInput bottomSensor;

    public final XLaserCAN distanceSensor;


    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf,
                             ElectricalContract contract, XDigitalInput.XDigitalInputFactory xDigitalInputFactory,
                             XLaserCAN.XLaserCANFactory xLaserCANFactory) {

        this.contract = contract;

        this.elevatorTargetHeight = Inches.of(0);
        this.distanceFromTargetHeight = Feet.of(0);

        pf.setPrefix(this);
        //to be tuned
        this.metersPerRotation = pf.createPersistentProperty("MetersPerRotation", 3);

        //these are not real measured heights yet, just placeholders
        l1Height = pf.createPersistentProperty("l1Height", 1);
        l2Height = pf.createPersistentProperty("l2Height", 2);
        l3Height = pf.createPersistentProperty("l3Height", 3);
        l4Height = pf.createPersistentProperty("l4Height", 4);
        humanLoadHeight = pf.createPersistentProperty("humanLoadHeight", 2);
        baseHeight = pf.createPersistentProperty("baseHeight", 0);

        if(contract.isElevatorReady()){
            this.masterMotor = motorFactory.create(contract.getElevatorMotor(), this.getPrefix(), "ElevatorMotor");
            this.registerDataFrameRefreshable(masterMotor);
        }
        if (contract.isElevatorBottomSensorReady()){
            this.bottomSensor= xDigitalInputFactory.create(contract.getElevatorBottomSensor(), this.getPrefix());
        }else{
            this.bottomSensor=null;
        }

        if (contract.isElevatorDistanceSensorReady()) {
            this.distanceSensor = xLaserCANFactory.create(contract.getElevatorDistanceSensor(), this.getPrefix());
            registerDataFrameRefreshable(distanceSensor);
        } else {
            this.distanceSensor = null;
        }
    }


    //will implement logic later
    @Override
    public void setPower(double power) {
        if(contract.isElevatorReady()){
            masterMotor.setPower(power);
        }
    }

    @Override
    public Distance getCurrentValue() {
        Distance currentHeight = Inches.of(0);
        if (contract.isElevatorReady()){
            currentHeight = Meters.of(this.masterMotor.getPosition().in(Rotations) * metersPerRotation.get()); //hastily written code will clean up later
        }
        return currentHeight;
    }

    public LinearVelocity getCurrentVelocity() {
        return MetersPerSecond.of(masterMotor.getVelocity().in(RotationsPerSecond) * metersPerRotation.get());
    }

    @Override
    public Distance getTargetValue() {
       return elevatorTargetHeight;
    }

    @Override
    public void setTargetValue(Distance value) {
       elevatorTargetHeight = value;
    }

    public void setTargetHeight(ElevatorGoals value){
        switch (value){
            case ScoreL1 -> setTargetValue(Feet.of(l1Height.get()));
            case ScoreL2 -> setTargetValue(Feet.of(l2Height.get()));
            case ScoreL3 -> setTargetValue(Feet.of(l3Height.get()));
            case ScoreL4 -> setTargetValue(Feet.of(l4Height.get()));
            case HumanLoad -> setTargetValue(Feet.of(humanLoadHeight.get()));
            case ReturnToBase -> setTargetValue(Feet.of(baseHeight.get()));
            default -> setTargetValue(Feet.of(baseHeight.get()));
        }
    }

    public boolean isTouchingBottom(){
        if (contract.isElevatorBottomSensorReady()){
            return this.bottomSensor.get();
        }
        return false;
    }

    public void setCalibrated(boolean calibrated){
        isCalibrated = calibrated;
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    private Distance getRawDistance() {
        if (contract.isElevatorDistanceSensorReady()) {
            return distanceSensor.getDistance();
        } else {
            return Meter.of(0);
        }
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Distance target1, Distance target2) {
        return target1.isEquivalent(target2);
    }

    public void periodic(){
        if (contract.isElevatorReady()){
            masterMotor.periodic();
        }
        aKitLog.record("ElevatorTargetHeight-m",elevatorTargetHeight);
        aKitLog.record("ElevatorCurrentHeight-m",getCurrentValue().in(Meters));
        aKitLog.record("ElevatorBottomSensor",this.isTouchingBottom());
        aKitLog.record("ElevatorDistanceSensor-m",getRawDistance().in(Meters));
    }


}
