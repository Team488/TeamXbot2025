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

    final DoubleProperty elevatorTargetHeight;
    final DoubleProperty currentHeight;

    //assuming we'll have a master and follower motors
    public XCANMotorController masterMotor;

    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf, ElectricalContract contract){

        this.elevatorPower = pf.createPersistentProperty("Elevator Power", 0.5);
        this.contract = contract;

        //was going to make this ephemeral but cant find it
        this.elevatorTargetHeight = pf.createPersistentProperty("Elevator Target", 0.0);
        this.currentHeight = pf.createPersistentProperty("Current Height", 0.0);


        if(contract.isElevatorReady()){
            this.masterMotor = motorFactory.create(contract.getElevatorMotor(), this.getPrefix(), "Elevator Motor");
        }
    }

    //will implement logic later
    @Override
    public void setPower(Distance power) {

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
        return null;
    }

    @Override
    public Distance getTargetValue() {
        return null;
    }

    @Override
    public void setTargetValue(Distance value) {

    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Distance target1, Distance target2) {
        return false;
    }

    @Override
    public void refreshDataFrame() {
        masterMotor.refreshDataFrame();
    }

    public void periodic(){
        masterMotor.periodic();
    }


}
