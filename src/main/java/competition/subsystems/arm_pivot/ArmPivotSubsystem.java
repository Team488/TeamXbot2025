package competition.subsystems.arm_pivot;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

@Singleton
public class ArmPivotSubsystem extends BaseSetpointSubsystem<Angle> {
    public final XCANMotorController armMotor;
    Angle targetAngle = Degrees.of(0);
    ElectricalContract electricalContract;
    DoubleProperty degreesPerRotations;
    double rotationsAtZero;
    boolean isCalibrated = false;

    CoralScorerSubsystem coralScorerSubsystem;

    @Inject
    public ArmPivotSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                             ElectricalContract electricalContract, PropertyFactory propertyFactory) {

        propertyFactory.setPrefix(this);
        this.electricalContract = electricalContract;
        if (electricalContract.isArmPivotMotorReady()) {
            this.armMotor = xcanMotorControllerFactory.create(electricalContract.getArmPivotMotor(),
                    getPrefix(), "ArmPivotMotor");
            this.registerDataFrameRefreshable(this.armMotor);
        } else {
            this.armMotor = null;
        }

        this.degreesPerRotations = propertyFactory.createPersistentProperty("Degrees Per Rotations", 1);
    }



    @Override
    public Angle getCurrentValue() {
        double currentAngle = getMotorPositionFromZeroOffset().in(Rotations) * degreesPerRotations.get();
        return Degrees.of(currentAngle);
    }

    private Angle getMotorPositionFromZeroOffset() {
        return getMotorPosition().minus(Rotations.of(rotationsAtZero));
    }

    private Angle getMotorPosition() {
        if (electricalContract.isArmPivotMotorReady()) {
            return this.armMotor.getPosition();
        }
        return Rotations.of(0);
    }

    public void  preventNegative() {
        if (coralScorerSubsystem.hasCoral()) {

        }
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
        if (electricalContract.isArmPivotMotorReady() && coralScorerSubsystem.hasCoral()) {
            if (power < 0) {
                power = 0;
            }
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
}