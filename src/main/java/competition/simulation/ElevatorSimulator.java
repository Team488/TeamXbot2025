package competition.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.elevator.ElevatorMechanism;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;

@Singleton
public class ElevatorSimulator {
    final double loopPeriodSec = 0.02; 

    protected final AKitLogger aKitLog;

    final ElevatorSim elevatorSim;
    final DCMotor elevatorGearBox = DCMotor.getKrakenX60(2);

    final ElevatorSubsystem elevatorSubsystem;
    final MockCANMotorController motor;
    // Placeholder for getting real elevator voltage from the ElevatorSubsystem when it exists
    final ElevatorMechanism elevatorMechanism;
    public boolean elevatorIsAtBottom = true;

    @Inject
    public ElevatorSimulator(ElevatorMechanism elevatorMechanism, ElevatorSubsystem elevatorSubsystem) {
        aKitLog = new AKitLogger("Simulator/");
        this.elevatorMechanism = elevatorMechanism;
        this.elevatorSubsystem = elevatorSubsystem;
        this.motor = (MockCANMotorController)elevatorSubsystem.masterMotor;

        this.elevatorSim = new ElevatorSim(
            elevatorGearBox,
            ElevatorSimConstants.elevatorGearing,
            ElevatorSimConstants.carriageMass,
            ElevatorSimConstants.elevatorDrumRadius,
            ElevatorSimConstants.minElevatorHeightMeters,
            ElevatorSimConstants.maxElevatorHeightMeters,
            true,
            0,
            0.0,
            0.0);
    }

    public void update() {
        this.elevatorSim.setInputVoltage(this.motor.getPower());

        this.elevatorSim.update(loopPeriodSec);
        
        // Read out the new elevator position for rendering
        var elevatorCurrentHeight = Meters.of(this.elevatorSim.getPositionMeters());
        // TODO: instead of setting the mechanism directly this should go via setting the encoder ticks on the elevator subsystem when it exists
        this.elevatorMechanism.elevatorHeight = elevatorCurrentHeight;
        this.motor.setPosition(Rotations.of(elevatorCurrentHeight.in(Meters) * ElevatorSimConstants.rotationsPerMeterHeight));
        
        // this would be used to simulate the bottom position sensor being triggered
        this.elevatorIsAtBottom = elevatorCurrentHeight.in(Meters) <= ElevatorSimConstants.elevatorBottomSensorTriggerHeight;
        aKitLog.record("FieldSimulation/ElevatorHeight-Meters", elevatorCurrentHeight.in(Meters));
        aKitLog.record("FieldSimulation/ElevatorBottomSensorTriggered", this.elevatorIsAtBottom);
    }    
}
