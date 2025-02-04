package competition.simulation.coral_arm;

import javax.inject.Inject;

import competition.simulation.MotorInternalPIDHelper;
import competition.simulation.SimulationConstants;
import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;
import xbot.common.math.PIDManager;
import xbot.common.properties.PropertyFactory;

public class CoralArmSimulator {
    final DCMotor motor = DCMotor.getKrakenX60(1);
    final SingleJointedArmSim armSim;
    final PIDManager pidManager;

    final CoralArmPivotSubsystem armPivotSubsystem;
    final MockCANMotorController armMotor;

    @Inject
    public CoralArmSimulator(CoralArmPivotSubsystem armPivotSubsystem, PIDManager.PIDManagerFactory pidManagerFactory, PropertyFactory pf) {
        pf.setPrefix("CoralArmSimulator");
        this.pidManager = pidManagerFactory.create(pf.getPrefix() + "/CANMotorPositionalPID", 0.01, 0.001, 0.0, 0.0, 1.0, -1.0);
        this.armPivotSubsystem = armPivotSubsystem;
        this.armMotor = (MockCANMotorController) armPivotSubsystem.armMotor;

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

        // invert power because the simulated arm is going "backwards"
        armSim.setInput(this.armMotor.getPower() * RobotController.getBatteryVoltage() * -1.0);
        armSim.update(SimulationConstants.loopPeriodSec); // 20ms

        // Read out the new arm position for rendering
        var armRelativeAngle = getArmAngle();

        var armMotorRotations = armRelativeAngle.in(Radians) / CoralArmSimConstants.armEncoderAnglePerRotation.in(Radians);
        armMotor.setPosition(Rotations.of(armMotorRotations));
    }

    public Angle getArmAngle() {
        // convert from the armSim frame of reference to our actual arm frame of
        // reference where the bottom is 0' and the top is 125'
        var armSimAngle = Radians.of(armSim.getAngleRads());

        return armSimAngle.minus(CoralArmSimConstants.maxAngleRads).times(-1);
    }

    public boolean isAtCollectionAngle() {
        return getArmAngle().isNear(Degrees.of(0), Degrees.of(4));
    }
}
