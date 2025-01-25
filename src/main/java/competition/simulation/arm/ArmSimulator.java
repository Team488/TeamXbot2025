package competition.simulation.arm;

import javax.inject.Inject;

import competition.simulation.SimulationConstants;
import competition.subsystems.elevator.ElevatorMechanism;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Kilograms;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSimulator {
    final DCMotor motor = DCMotor.getKrakenX60(1);
    final SingleJointedArmSim armSim;

    final ElevatorMechanism elevatorMechanism;

    // TODO: replace this with reading the arm voltage from the subsystem when it exists
    public double armVoltage = 0;

    @Inject
    public ArmSimulator(ElevatorMechanism elevatorMechanism) {
        this.elevatorMechanism = elevatorMechanism;

        this.armSim = new SingleJointedArmSim(
                motor,
                ArmSimConstants.armReduction,
                SingleJointedArmSim.estimateMOI(ArmSimConstants.armLength.in(Meters),
                        ArmSimConstants.armMass.in(Kilograms)),
                ArmSimConstants.armLength.in(Meters),
                ArmSimConstants.minAngleRads.in(Radians),
                ArmSimConstants.maxAngleRads.in(Radians),
                true,
                0,
                ArmSimConstants.armEncoderDistPerPulse.in(Meters),
                0.0 // Add noise with a std-dev of 1 tick
        );
    }

    public void update() {
        armSim.setInput(armVoltage);
        armSim.update(SimulationConstants.loopPeriodSec); // 20ms

        // TODO: This should instead fake the values to the arm encoder on the arm subsystem when it exists
        elevatorMechanism.armAngle = Radians.of(armSim.getAngleRads());
    }
}
