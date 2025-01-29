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
import xbot.common.math.PIDManager;
import xbot.common.math.PIDManager.PIDManagerFactory;
import edu.wpi.first.wpilibj.MockDigitalInput;

@Singleton
public class ElevatorSimulator {
    protected final AKitLogger aKitLog;

    final ElevatorSim elevatorSim;
    final DCMotor elevatorGearBox = DCMotor.getKrakenX60(2);
    final PIDManager pidManager;

    final ElevatorSubsystem elevatorSubsystem;
    final MockCANMotorController motor;
    final MockDigitalInput bottomSensor;
    final ElevatorMechanism elevatorMechanism;

    @Inject
    public ElevatorSimulator(ElevatorMechanism elevatorMechanism, ElevatorSubsystem elevatorSubsystem,
            PIDManagerFactory pidManagerFactory) {
        aKitLog = new AKitLogger("Simulator/");
        this.elevatorMechanism = elevatorMechanism;
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidManager = pidManagerFactory.create("ElevatorSimulationPositionalPID", 0.001, 0.0, 0.0, 0.0, -1.0, 1.0);
        this.motor = (MockCANMotorController) elevatorSubsystem.masterMotor;
        this.bottomSensor = (MockDigitalInput) elevatorSubsystem.bottomSensor;

        // init motor position to our random home value
        motor.setPosition(ElevatorSimConstants.rotationsAtZero);

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
        // based on the motor state, potentially run internal PID if need be
        if (motor.getControlMode() == MockCANMotorController.ControlMode.Position) {
            // run a simple pid to mimic the internal pid of the motor controller
            var targetPosition = motor.getTargetPosition();
            var currentPosition = motor.getPosition();
            var gravityFeedForward = 0.1; // constant force to fight gravity
            var output = pidManager.calculate(currentPosition.in(Rotations), targetPosition.in(Rotations))
                    + gravityFeedForward;
            motor.setPower(output);
        } else {
            pidManager.reset();
        }

        this.elevatorSim.setInputVoltage(this.motor.getPower() * RobotController.getBatteryVoltage());

        this.elevatorSim.update(SimulationConstants.loopPeriodSec);

        // Read out the new elevator position for rendering
        var elevatorCurrentHeight = Meters.of(this.elevatorSim.getPositionMeters());
        this.elevatorMechanism.elevatorHeight = elevatorCurrentHeight;
        // update the motor encoder position based on the elevator height, add in the
        // random from zero offset
        this.motor.setPosition(
                Rotations.of(elevatorCurrentHeight.in(Meters) * ElevatorSimConstants.rotationsPerMeterHeight)
                        .plus(ElevatorSimConstants.rotationsAtZero));

        // this would be used to simulate the bottom position sensor being triggered
        var elevatorIsAtBottom = elevatorCurrentHeight
                .in(Meters) <= ElevatorSimConstants.elevatorBottomSensorTriggerHeight;
        bottomSensor.setValue(elevatorIsAtBottom);
        aKitLog.record("FieldSimulation/ElevatorHeight-Meters", elevatorCurrentHeight.in(Meters));
        aKitLog.record("FieldSimulation/ElevatorBottomSensorTriggered", elevatorIsAtBottom);
    }

    public boolean isAtCollectionHeight() {
        return this.elevatorMechanism.elevatorHeight.isNear(Meters.of(0.0), 0.05);
    }
}
