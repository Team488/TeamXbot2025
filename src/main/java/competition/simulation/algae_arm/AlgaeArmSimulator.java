package competition.simulation.algae_arm;

import javax.inject.Inject;

import competition.simulation.MotorInternalPIDHelper;
import competition.simulation.SimulationConstants;
import competition.subsystems.algae_arm.AlgaeArmSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;
import xbot.common.math.PIDManager;
import xbot.common.properties.PropertyFactory;

public class AlgaeArmSimulator {
    final DCMotor motor = DCMotor.getKrakenX60(1);
    final SingleJointedArmSim armSim;
    final PIDManager pidManager;

    final AlgaeArmSubsystem armPivotSubsystem;
    final MockCANMotorController armMotor;

    final AKitLogger aKitLog;

    @Inject
    public AlgaeArmSimulator(AlgaeArmSubsystem armPivotSubsystem, PIDManager.PIDManagerFactory pidManagerFactory, PropertyFactory pf) {
        pf.setPrefix("Simulator/AlgaeArm");
        aKitLog = new AKitLogger("Simulator/AlgaeArm/");
        this.pidManager = pidManagerFactory.create(pf.getPrefix() + "/CANMotorPositionalPID", 0.01, 0.001, 0.0, 0.0, 1.0, -1.0);
        this.armPivotSubsystem = armPivotSubsystem;
        this.armMotor = (MockCANMotorController) armPivotSubsystem.armMotor;

        this.armSim = new SingleJointedArmSim(
                motor,
                AlgaeArmSimConstants.armReduction,
                SingleJointedArmSim.estimateMOI(AlgaeArmSimConstants.armLength.in(Meters),
                        AlgaeArmSimConstants.armMass.in(Kilograms)),
                AlgaeArmSimConstants.armLength.in(Meters),
                AlgaeArmSimConstants.minAngleRads.in(Radians),
                AlgaeArmSimConstants.maxAngleRads.in(Radians),
                true,
                AlgaeArmSimConstants.startingAngle.in(Radians));
    }

    public void update() {
        // based on the motor state, potentially run internal PID if need be
        MotorInternalPIDHelper.updateInternalPID(armMotor, pidManager);

        if(DriverStation.isEnabled()) {
            // invert power because the simulated arm is going "backwards"
            armSim.setInput(this.armMotor.getPower() * RobotController.getBatteryVoltage() * -1.0);
        } else {
            armSim.setInput(0.0);
        }
        
        armSim.update(SimulationConstants.loopPeriodSec); // 20ms

        // Read out the new arm position for rendering
        var armRelativeAngle = getArmAngle();
        aKitLog.record("armRawSimAngleDegrees",  Radians.of(armSim.getAngleRads()).in(Degrees));
        aKitLog.record("armRelativeAngleDegrees", armRelativeAngle.in(Degrees));

        var armMotorRotations = armRelativeAngle.in(Radians) / AlgaeArmSimConstants.armEncoderAnglePerRotation.in(Radians);
        armMotor.setPosition(Rotations.of(armMotorRotations));

        // TODO: simulate lower limit sensor triggered when arm is at 0' in relative terms
    }

    public Angle getArmAngle() {
        // TODO: convert from global frame of reference to 0' being down
        var armSimAngle = Radians.of(armSim.getAngleRads());

        return armSimAngle.plus(Degrees.of(90));
    }

    public boolean isAtCollectionAngle() {
        return getArmAngle().isNear(Degrees.of(0), Degrees.of(5));
    }
}
