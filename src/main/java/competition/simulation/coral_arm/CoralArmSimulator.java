package competition.simulation.coral_arm;

import javax.inject.Inject;

import competition.Robot;
import competition.electrical_contract.ElectricalContract;
import competition.simulation.MotorInternalPIDHelper;
import competition.subsystems.coral_arm.CoralArmSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MockDigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;
import xbot.common.controls.sensors.mock_adapters.MockDutyCycleEncoder;
import xbot.common.math.PIDManager;
import xbot.common.properties.PropertyFactory;

public class CoralArmSimulator {
    final DCMotor motor = DCMotor.getKrakenX60(1);
    final SingleJointedArmSim armSim;
    final PIDManager pidManager;
    final AKitLogger aKitLog;

    final CoralArmSubsystem armPivotSubsystem;
    final MockCANMotorController armMotor;
    final MockDutyCycleEncoder absoluteEncoder;
    final MockDigitalInput lowSensor;
    final ElectricalContract contract;

    @Inject
    public CoralArmSimulator(CoralArmSubsystem armPivotSubsystem, PIDManager.PIDManagerFactory pidManagerFactory, PropertyFactory pf, ElectricalContract contract) {
        pf.setPrefix("CoralArmSimulator");
        this.aKitLog = new AKitLogger("FieldSimulation/CoralArm");
        this.contract = contract;
        this.pidManager = pidManagerFactory.create(pf.getPrefix() + "/CANMotorPositionalPID", 0.2, 0.001, 0.0, 0.0, 1.0, -1.0);
        this.armPivotSubsystem = armPivotSubsystem;
        this.armMotor = (MockCANMotorController) armPivotSubsystem.armMotor;
        this.absoluteEncoder = (MockDutyCycleEncoder) armPivotSubsystem.armAbsoluteEncoder;
        this.lowSensor = (MockDigitalInput) armPivotSubsystem.lowSensor;

        this.armSim = new SingleJointedArmSim(
                motor,
                CoralArmSimConstants.armReduction,
                SingleJointedArmSim.estimateMOI(CoralArmSimConstants.armLength.in(Meters),
                        CoralArmSimConstants.armMass.in(Kilograms)),
                CoralArmSimConstants.armLength.in(Meters),
                CoralArmSimConstants.minAngleRads.in(Radians),
                CoralArmSimConstants.maxAngleRads.in(Radians),
                true,
                CoralArmSimConstants.startingAngle.in(Radians));
    }

    public void update() {
        // based on the motor state, potentially run internal PID if need be
        MotorInternalPIDHelper.updateInternalPID(armMotor, pidManager);

        if(DriverStation.isEnabled()) {
            armSim.setInput(this.armMotor.getPower() * RobotController.getBatteryVoltage() * -1.0);
        } else {
            armSim.setInput(0.0);
        }
        // invert power because the simulated arm is going "backwards"
        armSim.update(Robot.LOOP_INTERVAL);

        // Read out the new arm position for rendering
        var armRelativeAngle = getArmAngle();
        aKitLog.record("armAngle", armRelativeAngle.in(Degrees));

        var armMotorRotations = armRelativeAngle.in(Radians) / CoralArmSimConstants.armEncoderAnglePerRotation.in(Radians);
        armMotor.setPosition(Rotations.of(armMotorRotations));

        if(contract.isCoralArmPivotAbsoluteEncoderReady()) {
            absoluteEncoder.setRawPosition(getAbsoluteEncoderPosition(getArmAngle(), 0.0,
                    armPivotSubsystem.rangeOfMotionDegrees.get() / 360).in(Rotations) + 0.5);
        }

        // if the arm angle is lower than 10.8 degrees it will return true, otherwise return false
        lowSensor.setValue(getArmAngle().in(Degrees) < 10.8);
    }

    public Angle getArmAngle() {
        // convert from the armSim frame of reference to our actual arm frame of
        // reference where the bottom is 0' and the top is 125'
        var armSimAngle = Radians.of(armSim.getAngleRads());

        return armSimAngle.minus(CoralArmSimConstants.angleAtRobotZero).times(-1);
    }

    public boolean isAtCollectionAngle() {
        return getArmAngle().isNear(Degrees.of(0), Degrees.of(4));
    }

    // minPosition and maxPosition are rotations normalized to [0,1]
    public static Angle getAbsoluteEncoderPosition(Angle armAngle, double minPosition, double maxPosition) {
        double currentPosition = armAngle.in(Degrees) / 125;
        double absoluteEncoderPosition;
        if (maxPosition > minPosition) {
            absoluteEncoderPosition = minPosition + currentPosition;
            if (absoluteEncoderPosition > 1) {
                absoluteEncoderPosition = maxPosition + currentPosition - 1;
            }
        }
        else {
            if (currentPosition < maxPosition) {
                absoluteEncoderPosition = maxPosition - currentPosition;
            }
            else if (currentPosition > minPosition) {
                absoluteEncoderPosition = 1 - currentPosition - maxPosition;
            }
            else {
                absoluteEncoderPosition = currentPosition + minPosition - 1;
            }
        }

        return Degrees.of(absoluteEncoderPosition * 360);
    }

}