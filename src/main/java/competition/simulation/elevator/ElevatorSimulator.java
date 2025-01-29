package competition.simulation.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.simulation.SimulationConstants;
import competition.subsystems.elevator.ElevatorMechanism;
import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;
import edu.wpi.first.wpilibj.MockDigitalInput;


@Singleton
public class ElevatorSimulator {
    protected final AKitLogger aKitLog;

    final ElevatorSim elevatorSim;
    final DCMotor elevatorGearBox = DCMotor.getKrakenX60(2);

    final ElevatorSubsystem elevatorSubsystem;
    final MockCANMotorController motor;
    final MockDigitalInput bottomSensor;
    final ElevatorMechanism elevatorMechanism;

    @Inject
    public ElevatorSimulator(ElevatorMechanism elevatorMechanism, ElevatorSubsystem elevatorSubsystem) {
        aKitLog = new AKitLogger("Simulator/");
        this.elevatorMechanism = elevatorMechanism;
        this.elevatorSubsystem = elevatorSubsystem;
        this.motor = (MockCANMotorController)elevatorSubsystem.masterMotor;
        this.bottomSensor = (MockDigitalInput)elevatorSubsystem.bottomSensor;

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
        this.elevatorSim.setInputVoltage(this.motor.getPower() * RobotController.getBatteryVoltage());

        this.elevatorSim.update(SimulationConstants.loopPeriodSec);
        
        // Read out the new elevator position for rendering
        var elevatorCurrentHeight = Meters.of(this.elevatorSim.getPositionMeters());
        // TODO: instead of setting the mechanism directly this should go via setting the encoder ticks on the elevator subsystem when it exists
        this.elevatorMechanism.elevatorHeight = elevatorCurrentHeight;
        this.motor.setPosition(Rotations.of(elevatorCurrentHeight.in(Meters) * ElevatorSimConstants.rotationsPerMeterHeight));
        
        // this would be used to simulate the bottom position sensor being triggered
        var elevatorIsAtBottom = elevatorCurrentHeight.in(Meters) <= ElevatorSimConstants.elevatorBottomSensorTriggerHeight;
        bottomSensor.setValue(elevatorIsAtBottom);
        aKitLog.record("FieldSimulation/ElevatorHeight-Meters", elevatorCurrentHeight.in(Meters));
        aKitLog.record("FieldSimulation/ElevatorBottomSensorTriggered", elevatorIsAtBottom);
    }

    public boolean isAtCollectionHeight() {
        return this.elevatorMechanism.elevatorHeight.isNear(Meters.of(0.0), 0.05);
    }    
}
