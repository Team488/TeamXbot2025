package competition.subsystems;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class AlgaeCollectionSubsystem extends BaseSubsystem {
    public final XCANMotorController motor;

    public DoubleProperty intakePower;
    public DoubleProperty outputPower;

    @Inject
    public AlgaeCollectionSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory, ElectricalContract electricalContract, PropertyFactory propertyFactory) {
        this.motor = xcanMotorControllerFactory.create(electricalContract.getAlgaeCollectionMotor(), getPrefix(), "AlgaeMotor");

        this.intakePower = propertyFactory.createPersistentProperty("Intake Power", .1);
        this.outputPower = propertyFactory.createPersistentProperty("Output Power", .1);
    }

    public void intake() {
        motor.setPower(intakePower.get());
    }

    public void output() {
        motor.setPower(outputPower.get());
    }

    public void stop() {
        motor.setPower(0);
    }
}

