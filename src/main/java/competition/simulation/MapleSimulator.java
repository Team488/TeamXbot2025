package competition.simulation;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.elevator.ElevatorMechanism;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.sensors.mock_adapters.MockGyro;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.time.Duration;

import javax.inject.Inject;
import javax.inject.Singleton;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

@Singleton
public class MapleSimulator implements BaseSimulator {
    final PoseSubsystem pose;
    final DriveSubsystem drive;
    final double loopPeriodSec = 0.02; 

    protected final AKitLogger aKitLog;

    // maple-sim stuff
    final DriveTrainSimulationConfig config;
    final SimulatedArena arena;
    final SelfControlledSwerveDriveSimulation swerveDriveSimulation;

    // elevator stuff
    // Simulation classes help us simulate what's going on, including gravity.
    final ElevatorSim elevatorSim;
    final DCMotor elevatorGearBox = DCMotor.getKrakenX60(2);

    // Placeholder for getting real elevator voltage from the ElevatorSubsystem
    public double elevatorVoltage = 0;
    final ElevatorMechanism elevatorMechanism;

    @Inject
    public MapleSimulator(PoseSubsystem pose, DriveSubsystem drive, ElevatorMechanism elevatorMechanism) {
        this.pose = pose;
        this.drive = drive;
        this.elevatorMechanism = elevatorMechanism;

        aKitLog = new AKitLogger("Simulator/");

        /**
         * MapleSim arena and drive setup
         */
        arena = SimulatedArena.getInstance();
        arena.resetFieldForAuto();
        // TODO: custom things to provide here like motor ratios and what have you
        config = DriveTrainSimulationConfig.Default().withCustomModuleTranslations(new Translation2d[] {
                drive.getFrontLeftSwerveModuleSubsystem().getModuleTranslation(),
                drive.getFrontRightSwerveModuleSubsystem().getModuleTranslation(),
                drive.getRearLeftSwerveModuleSubsystem().getModuleTranslation(),
                drive.getRearRightSwerveModuleSubsystem().getModuleTranslation()
        });

        // starting middle ish of the field on blue
        var startingPose = new Pose2d(6, 4, new Rotation2d());

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.swerveDriveSimulation = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, startingPose));
        // Tell the robot it's starting in the same spot
        pose.setCurrentPoseInMeters(startingPose);

        arena.addDriveTrainSimulation(swerveDriveSimulation.getDriveTrainSimulation());

        /**
         * Elevator sim setup
         */
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
        this.updateDriveSimulation();
        this.updateElevatorSimulation();
    }

    protected void updateElevatorSimulation() {
        this.elevatorSim.setInputVoltage(elevatorVoltage);

        this.elevatorSim.update(loopPeriodSec);
        
        // Read out the new elevator position for rendering
        this.elevatorMechanism.elevatorHeight = Meters.of(this.elevatorSim.getPositionMeters());
    }

    protected void updateDriveSimulation() {
        // drive simulated robot from requested robot commands
        swerveDriveSimulation.runSwerveStates(new SwerveModuleState[] {
                drive.getFrontLeftSwerveModuleSubsystem().getTargetState(),
                drive.getFrontRightSwerveModuleSubsystem().getTargetState(),
                drive.getRearLeftSwerveModuleSubsystem().getTargetState(),
                drive.getRearRightSwerveModuleSubsystem().getTargetState()
        });

        // run the simulation
        arena.simulationPeriodic();
        swerveDriveSimulation.periodic();

        // read values back out from sim
        aKitLog.record(
                "FieldSimulation/Algae", arena.getGamePiecesArrayByType("Algae"));
        aKitLog.record(
                "FieldSimulation/Coral", arena.getGamePiecesArrayByType("Coral"));

        // this is where the robot really is in the sim
        aKitLog.record("FieldSimulation/RobotGroundTruthPose", swerveDriveSimulation.getActualPoseInSimulationWorld());
        // This one isn't crazy useful since we're doing our own odometry, but it's here
        // for completeness
        aKitLog.record("FieldSimulation/MapleOdometryPose", swerveDriveSimulation.getOdometryEstimatedPose());

        // tell the pose subystem about where the robot has moved based on odometry
        pose.ingestSimulatedSwerveModulePositions(swerveDriveSimulation.getLatestModulePositions());

        // update gyro reading from sim
        ((MockGyro) pose.imu).setYaw(this.swerveDriveSimulation.getOdometryEstimatedPose().getRotation().getDegrees());
        // if we want to give the gyro ground truth to make debugging other problems
        // easier swap to this:
        // ((MockGyro)
        // pose.imu).setYaw(this.swerveDriveSimulation.getActualPoseInSimulationWorld().getRotation().getDegrees());
    }

    @Override
    public void resetPosition(Pose2d pose) {
        arena.resetFieldForAuto();
        this.swerveDriveSimulation.getDriveTrainSimulation().setSimulationWorldPose(pose);
        this.pose.setCurrentPoseInMeters(pose);
    }
}
