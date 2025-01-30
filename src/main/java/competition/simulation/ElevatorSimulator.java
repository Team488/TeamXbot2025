package competition.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
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

    @Inject
    public ElevatorSimulator(ElevatorSubsystem elevatorSubsystem) {
        aKitLog = new AKitLogger("Simulator/");
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

    public Distance getCurrentHeight() {
        return Meters.of(this.elevatorSim.getPositionMeters());
    }

    public void update() {
        this.elevatorSim.setInputVoltage(this.motor.getPower() * RobotController.getBatteryVoltage());

        this.elevatorSim.update(SimulationConstants.loopPeriodSec);
        
        // Read out the new elevator position for rendering
        var elevatorCurrentHeight = getCurrentHeight();
        this.motor.setPosition(Rotations.of(elevatorCurrentHeight.in(Meters) * ElevatorSimConstants.rotationsPerMeterHeight));
        
        // this would be used to simulate the bottom position sensor being triggered
        var elevatorIsAtBottom = elevatorCurrentHeight.in(Meters) <= ElevatorSimConstants.elevatorBottomSensorTriggerHeight;
        bottomSensor.setValue(elevatorIsAtBottom);
        aKitLog.record("FieldSimulation/ElevatorHeight-Meters", elevatorCurrentHeight.in(Meters));
        aKitLog.record("FieldSimulation/ElevatorBottomSensorTriggered", elevatorIsAtBottom);
                // Record the robot relevative positive of the Elevator so AdvantageScope can render it correctly
        // NOTE: getting the arm to rotate correctly at the end of the elevator in AdvantageScope is a bit tricky, so ignoring that for now.
        aKitLog.record("ElevatorPose", new Pose3d(0, 0, elevatorCurrentHeight.in(Units.Meters), new Rotation3d()));
    }

    public boolean isAtCollectionHeight() {
        return getCurrentHeight().isNear(Meters.of(0.0), 0.05);
    }    
}
