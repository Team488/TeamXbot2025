package competition.simulation.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.simulation.MotorInternalPIDHelper;
import competition.simulation.SimulationConstants;
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
import xbot.common.controls.sensors.mock_adapters.MockLaserCAN;
import xbot.common.math.PIDManager;
import xbot.common.math.PIDManager.PIDManagerFactory;
import xbot.common.properties.PropertyFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MockDigitalInput;

@Singleton
public class ElevatorSimulator {
    protected final AKitLogger aKitLog;

    final ElevatorSim elevatorSim;
    final DCMotor elevatorGearBox = DCMotor.getKrakenX60(2);
    final PIDManager pidManager;
    final ElectricalContract electricalContract;
    final double gravityFeedForward = 0.02;
    final double laserNoiseInMeters = 0.01;

    final ElevatorSubsystem elevatorSubsystem;
    final MockCANMotorController motor;
    final MockDigitalInput bottomSensor;
    final MockLaserCAN laserSensor;

    @Inject
    public ElevatorSimulator(ElevatorSubsystem elevatorSubsystem, PIDManagerFactory pidManagerFactory,
            PropertyFactory pf, ElectricalContract electricalContract) {
        aKitLog = new AKitLogger("Simulator/");
        pf.setPrefix("ElevatorSimulator");
        this.electricalContract = electricalContract;
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidManager = pidManagerFactory.create("ElevatorSimulationPositionalPID", 0.05, 0.0, 0.0, 0.0, 1.0, -1.0);
        this.motor = (MockCANMotorController) elevatorSubsystem.masterMotor;
        this.bottomSensor = (MockDigitalInput) elevatorSubsystem.bottomSensor;
        this.laserSensor = (MockLaserCAN) elevatorSubsystem.distanceSensor;

        // init motor position to our random home value
        motor.setPosition(ElevatorSimConstants.rotationsAtZero);
        laserSensor.setDistance(ElevatorSimConstants.startingHeightMeters);

        this.elevatorSim = new ElevatorSim(
                elevatorGearBox,
                ElevatorSimConstants.elevatorGearing,
                ElevatorSimConstants.carriageMass,
                ElevatorSimConstants.elevatorDrumRadius,
                ElevatorSimConstants.minElevatorHeightMeters,
                ElevatorSimConstants.maxElevatorHeightMeters,
                true,
                ElevatorSimConstants.startingHeightMeters,
                0.0,
                0.0);
    }

    public Distance getCurrentHeight() {
        return Meters.of(this.elevatorSim.getPositionMeters());
    }

    public void update() {
        MotorInternalPIDHelper.updateInternalPID(motor, pidManager, gravityFeedForward);
        aKitLog.record("ElevatorMotorControlMode", motor.getControlMode());

        if(DriverStation.isEnabled()) {
            this.elevatorSim.setInputVoltage(this.motor.getPower() * RobotController.getBatteryVoltage());
        } else {
            this.elevatorSim.setInputVoltage(0);
        }

        this.elevatorSim.update(SimulationConstants.loopPeriodSec);

        // Read out the new elevator position for rendering
        var elevatorCurrentHeight = getCurrentHeight();

        // We'll set our laserSensor distance to our elevator height with some noise (to *mimic reality*)
        laserSensor.setDistance(elevatorCurrentHeight.in(Meters) + ((Math.random() - 0.5) * laserNoiseInMeters * 2));

        // update the motor encoder position based on the elevator height, add in the
        // random from zero offset
        var prevPosition = this.motor.getPosition();
        this.motor.setPosition(
                Rotations.of(elevatorCurrentHeight.in(Meters) * ElevatorSimConstants.rotationsPerMeterHeight)
                        .plus(ElevatorSimConstants.rotationsAtZero));
        // set the motors velocity based on change in position and the control loop time
        this.motor.setVelocity(prevPosition.minus(this.motor.getPosition()).per(Second).times(SimulationConstants.loopPeriodSec));

        // this would be used to simulate the bottom position sensor being triggered
        var elevatorIsAtBottom = elevatorCurrentHeight
                .in(Meters) <= ElevatorSimConstants.elevatorBottomSensorTriggerHeight;
        if(bottomSensor != null) {
            bottomSensor.setValue(elevatorIsAtBottom);
        }
        aKitLog.record("FieldSimulation/ElevatorHeight-Meters", elevatorCurrentHeight.in(Meters));
        aKitLog.record("FieldSimulation/ElevatorBottomSensorTriggered", elevatorIsAtBottom);
        // Record the robot relevative positive of the Elevator so AdvantageScope can render it correctly
        // NOTE: getting the arm to rotate correctly at the end of the elevator in AdvantageScope is a bit tricky, so ignoring that for now.
        aKitLog.record("FieldSimulation/ElevatorPose", new Pose3d(0, 0, elevatorCurrentHeight.in(Units.Meters), new Rotation3d()));
    }

    public boolean isAtCollectionHeight() {
        return getCurrentHeight().isNear(Meters.of(0.0), Meters.of(0.05));
    }    
}
