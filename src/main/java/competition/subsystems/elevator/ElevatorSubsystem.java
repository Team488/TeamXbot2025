package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Distance;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

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

    private boolean isCalibrated;

    public Distance elevatorTargetHeight;
    final Distance distanceFromTargetHeight;
    final Distance currentHeight;

    //assuming we'll have a master and follower motors
    public XCANMotorController masterMotor;

    //important heights
    public final Distance l1Height;
    public final Distance l2Height;
    public final Distance l3Height;
    public final Distance l4Height;
    public final Distance humanLoadHeight;
    public final Distance returnToBaseHeight;

    public final XDigitalInput bottomSensor;


    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf,
                             ElectricalContract contract, XDigitalInput.XDigitalInputFactory xDigitalInputFactory){

        this.contract = contract;

        this.elevatorTargetHeight = Feet.of(0);
        this.distanceFromTargetHeight = Feet.of(0);
        this.currentHeight = Inches.of(0);

        //these are not real measured heights yet, just placeholders
        l1Height = Feet.of(3);
        l2Height = Feet.of(4);
        l3Height = Feet.of(5);
        l4Height = Feet.of(6);
        humanLoadHeight = Feet.of(3);
        returnToBaseHeight = Feet.of(2);

        if(contract.isElevatorReady()){
            this.masterMotor = motorFactory.create(contract.getElevatorMotor(), this.getPrefix(), "Elevator Motor");
        }
        if (contract.isElevatorBottomSensorReady()){
            this.bottomSensor= xDigitalInputFactory.create(contract.getElevatorBottomSensor(), "Elevator Bottom Sensor0");
        }else{
            this.bottomSensor=null;
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

    public boolean touchBottom(){
        if (contract.isElevatorBottomSensorReady()){
            return this.bottomSensor.get();
        }
        return false;
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
        masterMotor.refreshDataFrame();
    }

    public void periodic(){
        masterMotor.periodic();
        aKitLog.record("ElevatorTargetHeight",elevatorTargetHeight);
    }


}
