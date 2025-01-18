package competition.subsystems.elevator;

import competition.electrical_contract.Contract2025;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANTalon;
import xbot.common.injection.electrical_contract.CANTalonInfo;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ElevatorSubsystem extends BaseSubsystem {

    public enum ElevatorGoal{
        //common TargetHeights
    }
    double defaultElevatorPower;
    final DoubleProperty elevatorPower;

    //assuming we'll have a master and follower motors
    public XCANTalon master;
    public XCANTalon follower;


    @Inject
    public ElevatorSubsystem(XCANTalon.XCANTalonFactory talonFactory, PropertyFactory pf, Contract2025 contract){

        defaultElevatorPower = 0.5;
        elevatorPower = pf.createPersistentProperty("Standard Power", 0.5);


    }

    public Object getCurrentValue() {
        return null;
    }


    public Object getTargetValue() {
        return null;
    }

    public void setTargetValue(Object value) {

    }

    public void setPower(double power) {
        elevatorPower.set(power);
    }

    public void rise(){
        setPower(elevatorPower.get());
    }

    public void lower(){
        setPower(-elevatorPower.get());
    }

    public boolean isCalibrated() {
        return false;
    }

}
