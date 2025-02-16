package competition.simulation;

import competition.electrical_contract.ElectricalContract;
import competition.simulation.algae_arm.AlgaeArmSimulator;
import competition.simulation.coral_arm.CoralArmSimulator;
import competition.simulation.coral_scorer.CoralScorerSimulator;
import competition.simulation.elevator.ElevatorSimulator;
import competition.simulation.reef.ReefSimulator;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.elevator.SuperstructureMechanism;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.sensors.mock_adapters.MockGyro;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import javax.inject.Inject;
import javax.inject.Singleton;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoral;
import xbot.common.injection.electrical_contract.XSwerveDriveElectricalContract;

@Singleton
public class MapleSimulator implements BaseSimulator {
    final PoseSubsystem pose;
    final DriveSubsystem drive;

    protected final AKitLogger aKitLog;

    final SuperstructureMechanism superstructureMechanism;

    // sub simulators ----------------------------
    final ElevatorSimulator elevatorSimulator;
    final CoralArmSimulator coralArmSimulator;
    final ReefSimulator reefSimulator;
    final CoralScorerSimulator coralScorerSimulator;
    final AlgaeArmSimulator algaeArmSimulator;
    final LightsSimulator lightsSimulator;

    final Distance humanLoadingDistanceThreshold = Meters.of(0.5);

    // maple-sim stuff ----------------------------
    final DriveTrainSimulationConfig config;
    final SimulatedArena arena;
    final SelfControlledSwerveDriveSimulation swerveDriveSimulation;

    @Inject
    public MapleSimulator(PoseSubsystem pose, DriveSubsystem drive, ElevatorSimulator elevatorSimulator,
                          CoralArmSimulator armSimulator, ReefSimulator reefSimulator,
                          CoralScorerSimulator coralScorerSimulator, AlgaeArmSimulator algaeArmSimulator,
                          LightsSimulator lightsSimulator, ElectricalContract electricalContract) {
        this.pose = pose;
        this.drive = drive;
        this.elevatorSimulator = elevatorSimulator;
        this.coralArmSimulator = armSimulator;
        this.reefSimulator = reefSimulator;
        this.coralScorerSimulator = coralScorerSimulator;
        this.algaeArmSimulator = algaeArmSimulator;
        this.lightsSimulator = lightsSimulator;
        this.superstructureMechanism = new SuperstructureMechanism();

        aKitLog = new AKitLogger("Simulator/");

        /**
         * MapleSim arena and drive setup
         */
        arena = SimulatedArena.getInstance();
        arena.resetFieldForAuto();
        // TODO: custom things to provide here like motor ratios and what have you
        config = DriveTrainSimulationConfig
                .Default()
                .withCustomModuleTranslations(new Translation2d[]{
                        drive.getFrontLeftSwerveModuleSubsystem().getModuleTranslation(),
                        drive.getFrontRightSwerveModuleSubsystem().getModuleTranslation(),
                        drive.getRearLeftSwerveModuleSubsystem().getModuleTranslation(),
                        drive.getRearRightSwerveModuleSubsystem().getModuleTranslation()
                })
                .withSwerveModule(() -> new SwerveModuleSimulation(new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60(1),
                        DCMotor.getKrakenX60(1),
                        electricalContract.getDriveGearRatio(),
                        electricalContract.getSteeringGearRatio(),
                        Volts.of(2.0),
                        Volts.of(2.0),
                        electricalContract.getDriveWheelDiameter().div(2),
                        KilogramSquareMeters.of(0.1),
                        0.899
                )));

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
        coralArmSimulator.update();
        reefSimulator.update();
        algaeArmSimulator.update();
        lightsSimulator.update();
        this.updateCoralLoadFromHumanPlayer();
        this.updateCoralScorerSensor();
        this.updateSuperstructureMechanism();
    }

    void updateSuperstructureMechanism() {
        superstructureMechanism.setElevatorHeight(elevatorSimulator.getCurrentHeight());
        superstructureMechanism.setCoralArmAngle(coralArmSimulator.getArmAngle());
        superstructureMechanism.setCoralInScorer(coralScorerSimulator.isCoralLoaded());
        superstructureMechanism.setAlgaeArmAngle(algaeArmSimulator.getArmAngle());
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
        if (coralScorerSimulator.isScoring()) {
            coralScorerSimulator.simulateCoralUnload();
            // get the front center of the robot as a proxy for where the stinger is in space
            var currentPose = this.getGroundTruthPose();
            var frontOfRobotOffset = new Translation2d(Inches.of(18).in(Meters), currentPose.getRotation());
            var frontOfRobot = currentPose.getTranslation().plus(frontOfRobotOffset);

            // TODO: more math around where the arm actually is in space and the orientation of the robot
            var elevatorBaseHeightM = 0.57;
            var aproxScorerTranslation3d = new Translation3d(
                    frontOfRobot.getX(),
                    frontOfRobot.getY(),
                    elevatorSimulator.getCurrentHeight().in(Meters) + elevatorBaseHeightM
            );

            var closetCoralKey = reefSimulator.findNearestCoral(aproxScorerTranslation3d);
            System.out.print("Closest coral key: " + closetCoralKey);
            var closetCoralPose = reefSimulator.getCoralPose(closetCoralKey);
            var coralAlreadyScored = reefSimulator.isCoralScored(closetCoralKey);

            double distanceToReef = aproxScorerTranslation3d.getDistance(closetCoralPose.getTranslation());

            System.out.println("Distance from closest reef: " + distanceToReef);
            if (distanceToReef > 0.3 || coralAlreadyScored) {
                if (coralAlreadyScored) {
                    System.out.println("Coral already scored, dropping on ground");
                } else {
                    System.out.println("Too far from reef, dropping on ground");
                }
                // we fail to score the coral, drop it on the ground
                ReefscapeCoral coral = new ReefscapeCoral(new Pose2d(frontOfRobot, currentPose.getRotation()));
                System.out.println("coral position: " + coral.getPoseOnField());
                arena.addGamePiece(coral);
            } else {
                reefSimulator.scoreCoral(closetCoralKey);
            }


        }
    }

    protected void updateCoralLoadFromHumanPlayer() {
        var coralScorerIsIntaking = coralScorerSimulator.isIntaking();
        var elevatorAtCollectionHeight = elevatorSimulator.isAtCollectionHeight();
        var armAtCollectionAngle = coralArmSimulator.isAtCollectionAngle();
        Pose2d[] coralStations = {Landmarks.BlueLeftCoralStationMid, Landmarks.BlueRightCoralStationMid};
        var currentPose = this.getGroundTruthPose();
        var robotNearHumanLoading = false;
        for (Pose2d station : coralStations) {
            if (currentPose.getTranslation().getDistance(station.getTranslation()) < humanLoadingDistanceThreshold.in(Meters)) {
                // verify robot angle aligns with the station
                var angleThresholdDegrees = 10;
                if (Math.abs(currentPose.getRotation().getDegrees() - station.getRotation().getDegrees()) < angleThresholdDegrees) {
                    robotNearHumanLoading = true;
                    break;
                }
            }
        }

        if (elevatorAtCollectionHeight && armAtCollectionAngle && coralScorerIsIntaking && robotNearHumanLoading) {
            coralScorerSimulator.simulateCoralLoad();
        }
    }

    protected void updateDriveSimulation() {
        // drive simulated robot from requested robot commands
        swerveDriveSimulation.runSwerveStates(new SwerveModuleState[]{
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
