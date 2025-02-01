package competition.simulation;

import competition.simulation.coral_arm.CoralArmSimulator;
import competition.simulation.coral_scorer.CoralScorerSimulator;
import competition.simulation.elevator.ElevatorSimulator;
import competition.simulation.reef.ReefSimulator;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.elevator.SuperstructureMechanism;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.sensors.mock_adapters.MockGyro;

import static edu.wpi.first.units.Units.Meters;

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

    final SuperstructureMechanism superstructureMechanism;

    // sub simulators ----------------------------
    final ElevatorSimulator elevatorSimulator;
    final CoralArmSimulator armSimulator;
    final ReefSimulator reefSimulator;
    final CoralScorerSimulator coralScorerSimulator;

    final Distance humanLoadingDistanceThreshold = Meters.of(0.5);

    // maple-sim stuff ----------------------------
    final DriveTrainSimulationConfig config;
    final SimulatedArena arena;
    final SelfControlledSwerveDriveSimulation swerveDriveSimulation;

    @Inject
    public MapleSimulator(PoseSubsystem pose, DriveSubsystem drive, ElevatorSimulator elevatorSimulator,
                          CoralArmSimulator armSimulator, ReefSimulator reefSimulator, CoralScorerSimulator coralScorerSimulator) {
        this.pose = pose;
        this.drive = drive;
        this.elevatorSimulator = elevatorSimulator;
        this.armSimulator = armSimulator;
        this.reefSimulator = reefSimulator;
        this.coralScorerSimulator = coralScorerSimulator;
        this.superstructureMechanism = new SuperstructureMechanism();

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
    }

    public void update() {
        this.updateDriveSimulation();
        elevatorSimulator.update();
        armSimulator.update();
        reefSimulator.update();
        this.updateCoralLoadFromHumanPlayer();
        this.updateCoralScorerSensor();
        this.updateSuperstructureMechanism();
    }
    
    void updateSuperstructureMechanism() {
        superstructureMechanism.setElevatorHeight(elevatorSimulator.getCurrentHeight());
        superstructureMechanism.setArmAngle(armSimulator.getArmAngle());
        superstructureMechanism.setCoralInScorer(coralScorerSimulator.isCoralLoaded());
        aKitLog.record("FieldSimulation/SuperstructureMechanism", superstructureMechanism.getMechanism());
    }

    protected void updateCoralScorerSensor() {
        // TODO:
        // if the elevator is at a reef height
        // && the arm is at the right angle
        // && there is a piece of coral in the scorer
        // && the scorer is ejecting
        // simulate scoring a piece of coral on the reef

        // for now this is just some quick hacky logic, whenever we outtake just score coral to closest spot on reef
        if(coralScorerSimulator.isScoring()) {
            coralScorerSimulator.simulateCoralUnload();
            var currentTranslation2d = this.getGroundTruthPose().getTranslation();
            // TODO: more math around where the arm actually is in space and the orientation of the robot
            var aproxElevatorTranslation3d = new Translation3d(currentTranslation2d.getX(), currentTranslation2d.getY(), 0.0);
            reefSimulator.scoreCoralNearestTo(aproxElevatorTranslation3d);
        }
    }

    protected void updateCoralLoadFromHumanPlayer() {
        var elevatorAtCollectionHeight = elevatorSimulator.isAtCollectionHeight();
        var armAtCollectionAngle = armSimulator.isAtCollectionAngle();
        var coralScorerIsIntaking = coralScorerSimulator.isIntaking();
        Pose2d[] coralStations = {Landmarks.BlueLeftCoralStationMid, Landmarks.BlueRightCoralStationMid};
        var currentPose = this.getGroundTruthPose();
        var robotNearHumanLoading = false; 
        for (Pose2d station : coralStations) {
            if (currentPose.getTranslation().getDistance(station.getTranslation()) < humanLoadingDistanceThreshold.in(Meters)) {
                robotNearHumanLoading = true;
                break;
            }
        }

        if (elevatorAtCollectionHeight && armAtCollectionAngle && coralScorerIsIntaking && robotNearHumanLoading) {
            coralScorerSimulator.simulateCoralLoad();
        }
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
        aKitLog.record("FieldSimulation/Robot", swerveDriveSimulation.getActualPoseInSimulationWorld());

        // tell the pose subystem about where the robot has moved based on odometry
        pose.ingestSimulatedSwerveModulePositions(swerveDriveSimulation.getLatestModulePositions());

        // update gyro reading from sim
        ((MockGyro) pose.imu).setYaw(this.swerveDriveSimulation.getOdometryEstimatedPose().getRotation().getDegrees());
    }

    @Override
    public void resetPosition(Pose2d pose) {
        arena.resetFieldForAuto();
        this.swerveDriveSimulation.getDriveTrainSimulation().setSimulationWorldPose(pose);
        this.pose.setCurrentPoseInMeters(pose);
    }

    @Override
    public Pose2d getGroundTruthPose() {
        return this.swerveDriveSimulation.getActualPoseInSimulationWorld();
    }
}
