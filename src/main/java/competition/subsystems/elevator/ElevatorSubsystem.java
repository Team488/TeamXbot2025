package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ElevatorSubsystem extends BaseSetpointSubsystem<Double> implements DataFrameRefreshable {

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
    public XCANMotorController followerMotor;


    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf, ElectricalContract contract){

        this.elevatorPower = pf.createPersistentProperty("Elevator Power", 0.5);
        this.contract = contract;

        //was going to make this ephemeral but cant find it
        this.elevatorTargetHeight = pf.createPersistentProperty("Elevator Target", 0.0);
        this.currentHeight = pf.createPersistentProperty("Current Height", 0.0);


        if(contract.isElevatorReady()){
            this.masterMotor = motorFactory.create(contract.getElevatorMaster(), this.getPrefix(), "Elevator Motor");
            this.followerMotor = motorFactory.create(contract.getElevatorFollower(), this.getPrefix(), "Elevator Motor");
        }
    }

    //will implement logic later
    public void setPower(Double power) {
        if (contract.isElevatorReady()) {
            masterMotor.setPower(power);
            followerMotor.setPower(power);
        }
    }

    public void rise(){
        setPower(elevatorPower.get());
    }

    public void lower(){
        setPower(-elevatorPower.get());
    }

    public void stop(){
        setPower(0.0);
    }


    @Override
    public Double getCurrentValue() {
        if (contract.isElevatorReady()){
            return this.currentHeight.get();
        }
        return 0.0;
    }

    @Override
    public void setTargetValue(Double targetHeight) {
        this.elevatorTargetHeight.set(targetHeight);
    }

    @Override
    public Double getTargetValue() {
        return this.elevatorTargetHeight.get();
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Double target1, Double target2) {
        return BaseSetpointSubsystem.areTwoDoublesEquivalent(target1, target2, 1);
    }

    @Override
    public void refreshDataFrame() {
        //to be implemented later
    }


}
