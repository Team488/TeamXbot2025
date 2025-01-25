package competition.subsystems.arm_pivot;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Angle;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

@Singleton
public class ArmPivotSubsystem extends BaseSetpointSubsystem<Angle> implements DataFrameRefreshable {
    public final XCANMotorController armMotor;
    Angle targetAngle;
    ElectricalContract electricalContract;
    DoubleProperty degreesPerRotations;
    double rotationsAtZero;
    boolean isCalibrated = false;

    @Inject
    public ArmPivotSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             ElectricalContract electricalContract, PropertyFactory propertyFactory) {

        propertyFactory.setPrefix(this);
        this.electricalContract = electricalContract;
        if (electricalContract.isArmPivotMotorReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getArmPivotMotor(),
                    getPrefix(), "ArmPivotMotor");
        } else {
            this.armMotor = null;
        }

        this.degreesPerRotations = propertyFactory.createPersistentProperty("Degrees Per Rotations", 1);
    }

    //

    @Override
    public Angle getCurrentValue() {
        double currentAngle = (this.armMotor.getPosition().in(Rotations) - rotationsAtZero) * degreesPerRotations.get();
        return Degrees.of(currentAngle);
    }

    @Override
    public Angle getTargetValue() {

        return targetAngle;
    }

    @Override
    public void setTargetValue(Angle value) {
        targetAngle = value;
    }

    @Override
    public void setPower(double power) {
        if (electricalContract.isArmPivotMotorReady()) {
            this.armMotor.setPower(power);
        }
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Angle target1, Angle target2) {
        return target1.isEquivalent(target2);
    }

    @Override
    public void refreshDataFrame() {
        if(electricalContract.isArmPivotMotorReady()) {
            this.armMotor.refreshDataFrame();
        }
    }
}