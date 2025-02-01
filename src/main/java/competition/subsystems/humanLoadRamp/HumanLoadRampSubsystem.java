package competition.subsystems.humanLoadRamp;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XServo;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton

public class HumanLoadRampSubsystem extends BaseSubsystem {
    public final XServo motor;
    public DoubleProperty extendedPower;
    public DoubleProperty retractedPower;
    public final ElectricalContract electricalContract;
    @Inject
    public HumanLoadRampSubsystem (XServo.XServoFactory xServoFactory, ElectricalContract electricalContract,
                                   PropertyFactory propertyFactory){

        if(electricalContract.isHumanLoadRampReady()){
            this.motor = xServoFactory.create( 1, "humanLoadRamp");
            this.registerDataFrameRefreshable(motor);
        }
        else {
            this.motor = null;


        }
        this.extendedPower= propertyFactory.createPersistentProperty("ExtendingPower",1);
        this.retractedPower=propertyFactory.createPersistentProperty("RetractingPower",1);
    }

    @Override
    public void Extend(){
        if (motor !=null){
            motor.set(extendedPower.get());
        }


    }


}
