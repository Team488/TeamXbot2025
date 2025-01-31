package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.properties.DoubleProperty;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

@Singleton
public class ElevatorSubsystem extends BaseSetpointSubsystem<Distance> {

    public enum ElevatorPowerRestrictionReason{
        FullPowerAvailable,
        BottomSensorHit,
        UpperSensorHit,
        Uncalibrated,
        AboveMaxHeight,
        BelowMinHeight
    }

    public enum ElevatorGoals{
        ScoreL2,
        ScoreL3,
        ScoreL4,
        HumanLoad,
        ReturnToBase,
    }

    final ElectricalContract contract;

    private boolean isCalibrated;
    private double calibrationOffset;
    //TODO: Add a calibration routine

    public Distance elevatorTargetHeight;
    final Distance distanceFromTargetHeight;

    final DoubleProperty metersPerRotation;
    public final DoubleProperty maxPowerWhenUncalibrated;

    public XCANMotorController masterMotor;

    //important heights
    public final DoubleProperty l2Height;
    public final DoubleProperty l3Height;
    public final DoubleProperty l4Height;
    public final DoubleProperty humanLoadHeight;
    public final DoubleProperty baseHeight;

    public final XDigitalInput bottomSensor;


    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf,
                             ElectricalContract contract, XDigitalInput.XDigitalInputFactory xDigitalInputFactory){

        this.contract = contract;

        this.calibrationOffset = 0.0;

        this.elevatorTargetHeight = Inches.of(0);
        this.distanceFromTargetHeight = Feet.of(0);

        pf.setPrefix(this);
        //to be tuned
        this.metersPerRotation = pf.createPersistentProperty("MetersPerRotation", 1);
        this.maxPowerWhenUncalibrated = pf.createPersistentProperty("maxPowerWhenUncalibrated", -0.01);

        //these are not real measured heights yet, just placeholders
        l2Height = pf.createPersistentProperty("l2Height-m", 1);
        l3Height = pf.createPersistentProperty("l3Height-m", 1.5);
        l4Height = pf.createPersistentProperty("l4Height-m", 2);
        humanLoadHeight = pf.createPersistentProperty("humanLoadHeight-m", 1);
        baseHeight = pf.createPersistentProperty("baseHeight-m", 0);

        if(contract.isElevatorReady()){
            this.masterMotor = motorFactory.create(contract.getElevatorMotor(), this.getPrefix(), "ElevatorMotor");
            this.registerDataFrameRefreshable(masterMotor);
        }
        if (contract.isElevatorBottomSensorReady()){
            this.bottomSensor= xDigitalInputFactory.create(contract.getElevatorBottomSensor(), "Elevator Bottom Sensor0");
        }else{
            this.bottomSensor=null;
        }

        setCalibrated(false);
    }


    //will implement logic later
    @Override
    public void setPower(double power) {
        if(contract.isElevatorReady()){
            if (!isCalibrated){
                power = maxPowerWhenUncalibrated.get();
            }
            masterMotor.setPower(power);
        }
    }

    public void calibrateHere(){
        calibrateAt(masterMotor.getPosition().in(Rotations));
    }

    public void calibrateAt(double lowestPosition){
        log.info("calibrating elevator with lowest position of " + lowestPosition);
        calibrationOffset = lowestPosition;

        masterMotor.setPower(maxPowerWhenUncalibrated.get());

        if(isTouchingBottom()){
            isCalibrated = true;
        }
    }

    @Override
    public Distance getCurrentValue() {
        Distance currentHeight = Inches.of(0);
        if (contract.isElevatorReady()){
            currentHeight = Meters.of(
                    (this.masterMotor.getPosition().in(Rotations) - calibrationOffset)* metersPerRotation.get()); //hastily written code will clean up later
        }
        return currentHeight;
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
            case ScoreL2 -> setTargetValue(Meters.of(l2Height.get()));
            case ScoreL3 -> setTargetValue(Meters.of(l3Height.get()));
            case ScoreL4 -> setTargetValue(Meters.of(l4Height.get()));
            case HumanLoad -> setTargetValue(Meters.of(humanLoadHeight.get()));
            case ReturnToBase -> setTargetValue(Meters.of(baseHeight.get()));
            default -> setTargetValue(Meters.of(baseHeight.get()));
        }
    }

    public boolean isTouchingBottom(){
        if (contract.isElevatorBottomSensorReady()){
            return true;
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
        aKitLog.record("isElevatorCalibrated", isCalibrated());
    }


}
