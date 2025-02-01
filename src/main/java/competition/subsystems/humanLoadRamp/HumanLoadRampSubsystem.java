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
    public final XServo servo;
    public DoubleProperty extendedPosition;
    public DoubleProperty retractedPosition;
    final ElectricalContract electricalContract;
    boolean retracted;
    @Inject
    public HumanLoadRampSubsystem (XServo.XServoFactory xServoFactory, ElectricalContract electricalContract,
                                   PropertyFactory propertyFactory) {
        this.electricalContract=electricalContract;
        if (electricalContract.isHumanLoadRampReady()) {
            this.servo = xServoFactory.create(1, "humanLoadRamp");
            this.registerDataFrameRefreshable(servo);
        } else {
            this.servo = null;
        }
        this.extendedPosition = propertyFactory.createPersistentProperty("ExtendingPosition", 1);
        this.retractedPosition = propertyFactory.createPersistentProperty("RetractingPosition", 0);
        retracted = true;
    }


    public void extend(){
        retracted=false;
        servo.set(extendedPosition.get());
    }

    public void retract(){
        retracted=true;
        servo.set(retractedPosition.get());
    }

    public boolean getItRetracted(){
        return retracted;
    }


}
