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

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

@Singleton
public class ElevatorSubsystem extends BaseSetpointSubsystem<Distance> implements DataFrameRefreshable {

    public enum ElevatorGoals{
        ScoreL1,
        ScoreL2,
        ScoreL3,
        ScoreL4,
        CoralCollection,
        ReturnToBase
    }

    final DoubleProperty elevatorPower;
    final ElectricalContract contract;

    private boolean isCalibrated;

    public Distance elevatorTargetHeight;
    final Distance distanceFromTargetHeight;
    final Distance currentHeight;

    //assuming we'll have a master and follower motors
    public XCANMotorController masterMotor;

    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf, ElectricalContract contract){

        this.elevatorPower = pf.createPersistentProperty("Elevator Power", 0.5);
        this.contract = contract;

        this.elevatorTargetHeight = Inches.of(0);
        this.distanceFromTargetHeight = Feet.of(0);
        this.currentHeight = Inches.of(0);

        if(contract.isElevatorReady()){
            this.masterMotor = motorFactory.create(contract.getElevatorMotor(), this.getPrefix(), "Elevator Motor");
        }
    }

    //will implement logic later
    @Override
    public void setPower(double power) {
        if(contract.isElevatorReady()){
            masterMotor.setPower(power);
        }
    }

    public void raise(){
        //setPower();
    }

    public void lower(){
       // setPower(-elevatorPower.get());
    }

    public void stop(){
       // setPower(0.0);
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
