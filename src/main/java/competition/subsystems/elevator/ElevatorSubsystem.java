package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import competition.simulation.elevator.ElevatorSimConstants;
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
    private double elevatorPositionOffset;
    private double elevatorScalingOffset;
    //TODO: Add a calibration routine

    public Distance elevatorTargetHeight;
    final Distance distanceFromTargetHeight;

    final DoubleProperty rotationsPerMeter;
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

        this.elevatorPositionOffset = 0.0;

        this.elevatorTargetHeight = Inches.of(0);
        this.distanceFromTargetHeight = Feet.of(0);

        pf.setPrefix(this);
        //to be tuned
        this.rotationsPerMeter = pf.createPersistentProperty("RotationsPerMeter", 1923);
        this.maxPowerWhenUncalibrated = pf.createPersistentProperty("maxPowerWhenUncalibrated", -0.05);

        //these are not real measured heights yet, just placeholders
        l2Height = pf.createPersistentProperty("l2Height-m", 0.5);
        l3Height = pf.createPersistentProperty("l3Height-m", 0.75);
        l4Height = pf.createPersistentProperty("l4Height-m", 1);
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

    public void markElevatorAsCalibratedAgainstLowerLimit(){
        isCalibrated = true;
        elevatorPositionOffset = this.masterMotor.getPosition().in(Rotations);
    }

    @Override
    public Distance getCurrentValue() {
        Distance currentHeight = Meters.of(0);
        if (contract.isElevatorReady()){
            currentHeight = Meters.of(
                    (this.masterMotor.getPosition().in(Rotations) - elevatorPositionOffset) / rotationsPerMeter.get());
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
            return getCurrentValue()
                    .in(Meters) <= ElevatorSimConstants.elevatorBottomSensorTriggerHeight;
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
