package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Distance;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

@Singleton
public class ElevatorSubsystem extends BaseSetpointSubsystem<Distance> implements DataFrameRefreshable {

    public enum ElevatorGoals{
        ScoreL1,
        ScoreL2,
        ScoreL3,
        ScoreL4,
        HumanLoad,
        ReturnToBase
    }

    final ElectricalContract contract;

    private boolean isCalibrated = true; //set to true just for testing purposes
    //TODO: Add a calibration routine

    public Distance elevatorTargetHeight;
    final Distance distanceFromTargetHeight;

    final DoubleProperty metersPerRotation;

    public XCANMotorController masterMotor;

    //important heights
    public final Distance l1Height;
    public final Distance l2Height;
    public final Distance l3Height;
    public final Distance l4Height;
    public final Distance humanLoadHeight;
    public final Distance returnToBaseHeight;

    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf, ElectricalContract contract){

        this.contract = contract;

        this.elevatorTargetHeight = Inches.of(0.1);
        this.distanceFromTargetHeight = Feet.of(0);

        pf.setPrefix(this);
        //to be tuned
        this.metersPerRotation = pf.createPersistentProperty("MetersPerRotation", 1.5);

        //these are not real measured heights yet, just placeholders
        l1Height = Feet.of(0.5);
        l2Height = Feet.of(1);
        l3Height = Feet.of(1.5);
        l4Height = Feet.of(2);
        humanLoadHeight = Feet.of(3);
        returnToBaseHeight = Feet.of(2);

        if(contract.isElevatorReady()){
            this.masterMotor = motorFactory.create(contract.getElevatorMotor(), this.getPrefix(), "ElevatorMotor");
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
            case ScoreL1 -> setTargetValue(l1Height);
            case ScoreL2 -> setTargetValue(l2Height);
            case ScoreL3 -> setTargetValue(l3Height);
            case ScoreL4 -> setTargetValue(l4Height);
            case HumanLoad -> setTargetValue(humanLoadHeight);
            case ReturnToBase -> setTargetValue(returnToBaseHeight);
            default -> setTargetValue(returnToBaseHeight);
        }
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Distance target1, Distance target2) {
        return target1.isEquivalent(target2);
    }

    @Override
    public void refreshDataFrame() {
        if(contract.isElevatorReady()){
            masterMotor.refreshDataFrame();
        }
    }

    public void periodic(){
        masterMotor.periodic();
        aKitLog.record("ElevatorTargetHeight",elevatorTargetHeight);
        aKitLog.record("ElevatorCurrentHeight",getCurrentValue().in(Meters));
    }


}
