package competition.subsystems.elevator;

import competition.electrical_contract.Contract2025;
import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.XCANTalon;
import xbot.common.injection.electrical_contract.CANTalonInfo;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ElevatorSubsystem extends BaseSubsystem {

    public enum ElevatorGoals{
        //common TargetHeights
    }

    final DoubleProperty elevatorPower;
    final ElectricalContract contract;

    private boolean isCalibrated;

    final DoubleProperty elevatorTargetHeight;

    //assuming we'll have a master and follower motors
    public XCANMotorController master;
    public XCANMotorController follower;


    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf, ElectricalContract contract){

        elevatorPower = pf.createPersistentProperty("Standard Power", 0.5);
        this.contract = contract;

        //was going to make this ephermeral but cant find it
        elevatorTargetHeight = pf.createPersistentProperty("Elevator Target", 0);

        if(contract.isElevatorReady()){
            this.master = motorFactory.create(contract.getElevatorMaster(), this.getPrefix(), "Elevator Motor");
            this.follower = motorFactory.create(contract.getElevatorFollower(), this.getPrefix(), "Elevator Motor");
        }
    }

    public void setTargetHeight(double height) {
        elevatorTargetHeight.set(height);
    }

    //will implement logic later
    public void setPower(double power) {
        if (contract.isElevatorReady()) {
            master.setPower(power);
            follower.setPower(power);
        }
    }

    public void rise(){
        setPower(elevatorPower.get());
    }

    public void lower(){
        setPower(-elevatorPower.get());
    }

    public void stop(){
        setPower(0);
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

}
