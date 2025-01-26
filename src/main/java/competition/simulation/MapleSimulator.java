package competition.simulation;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import xbot.common.advantage.AKitLogger;
import xbot.common.controls.sensors.mock_adapters.MockGyro;

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

    protected final AKitLogger aKitLog;

    // maple-sim stuff
    final DriveTrainSimulationConfig config;
    final SimulatedArena arena;
    final SelfControlledSwerveDriveSimulation swerveDriveSimulation;

    @Inject
    public MapleSimulator(PoseSubsystem pose, DriveSubsystem drive) {
        this.pose = pose;
        this.drive = drive;

        aKitLog = new AKitLogger("Simulator/");

        arena = SimulatedArena.getInstance();
        // TODO: custom things to provide here like motor ratios and what have you
        config = DriveTrainSimulationConfig.Default().withCustomModuleTranslations(new Translation2d[] {
                drive.getFrontLeftSwerveModuleSubsystem().getModuleTranslation(),
                drive.getFrontRightSwerveModuleSubsystem().getModuleTranslation(),
                drive.getRearLeftSwerveModuleSubsystem().getModuleTranslation(),
                drive.getRearRightSwerveModuleSubsystem().getModuleTranslation()
        });

        // middle ish of the field on blue
        var startingPose = new Pose2d(2, 2, new Rotation2d());

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.swerveDriveSimulation = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, startingPose));
        // Tell the robot it's starting in the same spot
        pose.setCurrentPoseInMeters(startingPose);

        arena.addDriveTrainSimulation(swerveDriveSimulation.getDriveTrainSimulation());
    }

    public void update() {
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
        aKitLog.record("RobotGroundTruthPose", swerveDriveSimulation.getActualPoseInSimulationWorld());
        aKitLog.record("MapleOdometryPose", swerveDriveSimulation.getOdometryEstimatedPose());

        // tell the pose subystem about where the robot has moved based on odometry
        pose.ingestSimulatedSwerveModulePositions(swerveDriveSimulation.getLatestModulePositions());
        
        // update gyro reading from sim
        ((MockGyro) pose.imu).setYaw(this.swerveDriveSimulation.getOdometryEstimatedPose().getRotation().getDegrees());
        // if we want to give the gyro ground truth to make debugging other problems easier swap to this:
        // ((MockGyro) pose.imu).setYaw(this.swerveDriveSimulation.getActualPoseInSimulationWorld().getRotation().getDegrees());
    }

    @Override
    public void resetPosition(Pose2d pose) {
        this.swerveDriveSimulation.getDriveTrainSimulation().setSimulationWorldPose(pose);
        this.pose.setCurrentPoseInMeters(pose);
    }
}
