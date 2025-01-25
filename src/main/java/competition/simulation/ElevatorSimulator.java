package competition.simulation;

import static edu.wpi.first.units.Units.Meters;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.elevator.ElevatorMechanism;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import xbot.common.advantage.AKitLogger;

@Singleton
public class ElevatorSimulator {
    protected final AKitLogger aKitLog;

    final ElevatorSim elevatorSim;
    final DCMotor elevatorGearBox = DCMotor.getKrakenX60(2);

    // Placeholder for getting real elevator voltage from the ElevatorSubsystem when it exists
    public double elevatorVoltage = 0;
    final ElevatorMechanism elevatorMechanism;
    public boolean elevatorIsAtBottom = true;

    @Inject
    public ElevatorSimulator(ElevatorMechanism elevatorMechanism) {
        aKitLog = new AKitLogger("Simulator/");
        this.elevatorMechanism = elevatorMechanism;

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
        this.elevatorSim.setInputVoltage(elevatorVoltage);

        this.elevatorSim.update(SimulationConstants.loopPeriodSec);
        
        // Read out the new elevator position for rendering
        var elevatorCurrentHeight = Meters.of(this.elevatorSim.getPositionMeters());
        // TODO: instead of setting the mechanism directly this should go via setting the encoder ticks on the elevator subsystem when it exists
        this.elevatorMechanism.elevatorHeight = elevatorCurrentHeight;
        // TODO: convert this height into an encoder tick count to set on the elevator subsystem
        // var simEncoderTicks = elevatorHeightToEncoderTicks(this.elevatorSim.getPositionMeters());
        // this.elevatorSubsystem.setSimulatedEncoderTicks(simEncoderTicks);
        
        // this would be used to simulate the bottom position sensor being triggered
        this.elevatorIsAtBottom = elevatorCurrentHeight.in(Meters) <= ElevatorSimConstants.elevatorBottomSensorTriggerHeight;
        aKitLog.record("FieldSimulation/ElevatorHeight-Meters", elevatorCurrentHeight.in(Meters));
        aKitLog.record("FieldSimulation/ElevatorBottomSensorTriggered", this.elevatorIsAtBottom);
    }

    static long elevatorHeightToEncoderTicks(double height) {
        // so much math
        return 0;
    }

    
}
