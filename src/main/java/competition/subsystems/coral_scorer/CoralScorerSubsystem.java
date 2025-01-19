package competition.subsystems.coral_scorer;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.XCANMotorControllerPIDProperties;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class CoralScorerSubsystem extends BaseSubsystem {
    public XCANMotorController motor = null;
    public DoubleProperty intakePower;
    public DoubleProperty scorePower;

    @Inject
    public CoralScorerSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                                ElectricalContract electricalContract, PropertyFactory propertyFactory) {
        if (electricalContract.isAlgaeCollectionReady()) {
            this.motor = xcanMotorControllerFactory.create(electricalContract.getAlgaeCollectionMotor(),
                    getPrefix(), "CoralScorer",
                    new XCANMotorControllerPIDProperties(1, 0, 0, 0, -1, 1));


        } else {
            this.motor = null;
        }

        this.intakePower = propertyFactory.createPersistentProperty("intakePower", .1);
        this.scorePower = propertyFactory.createPersistentProperty("scorerPower", .1);
    }

        public void intake() {
            if (motor != null){
                motor.setPower(intakePower.get());
            }
        }
        public void scorer(){
            if (motor != null){
                motor.setPower(scorePower.get());
            }
        }
        public void stop(){
            if (motor!=null){
                motor.setPower(0);
            }
        }

        //this.motor = motor;
    }

