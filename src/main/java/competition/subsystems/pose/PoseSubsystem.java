package competition.subsystems.pose;

import java.util.HashMap;
import java.util.Optional;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.drive.DriveSubsystem;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import xbot.common.controls.sensors.XGyro.XGyroFactory;
import xbot.common.math.WrappedRotation2d;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.pose.BasePoseSubsystem;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

@Singleton
public class PoseSubsystem extends BasePoseSubsystem {

    final SwerveDrivePoseEstimator onlyWheelsGyroSwerveOdometry;
    final SwerveDrivePoseEstimator fullSwerveOdometry;

    private final DriveSubsystem drive;
    private final AprilTagVisionSubsystem aprilTagVisionSubsystem;
    private final BooleanProperty useVisionAssistedPose;

    // only used when simulating the robot
    protected Optional<SwerveModulePosition[]> simulatedModulePositions = Optional.empty();

    @Inject
    public PoseSubsystem(XGyroFactory gyroFactory, PropertyFactory propManager, DriveSubsystem drive, AprilTagVisionSubsystem aprilTagVisionSubsystem) {
        super(gyroFactory, propManager);
        this.drive = drive;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;

        onlyWheelsGyroSwerveOdometry = initializeSwerveOdometry();
        fullSwerveOdometry = initializeSwerveOdometry();

        propManager.setPrefix(this);
        useVisionAssistedPose = propManager.createPersistentProperty("UseVisionAssistedPose", true);
    }

    @Override
    protected double getLeftDriveDistance() {
        return drive.getLeftTotalDistance();
    }

    @Override
    protected double getRightDriveDistance() {
        return drive.getRightTotalDistance();
    }


    private SwerveDrivePoseEstimator initializeSwerveOdometry() {
        return new SwerveDrivePoseEstimator(
                drive.getSwerveDriveKinematics(),
                getCurrentHeading(),
                getSwerveModulePositions(),
                new Pose2d());
    }

    @Override
    protected void updateOdometry() {
        // Update pose estimators
        onlyWheelsGyroSwerveOdometry.update(
                this.getCurrentHeading(),
                getSwerveModulePositions()
        );
        aKitLog.record("WheelsOnlyEstimate", onlyWheelsGyroSwerveOdometry.getEstimatedPosition());

        fullSwerveOdometry.update(
                this.getCurrentHeading(),
                getSwerveModulePositions()
        );
        this.aprilTagVisionSubsystem.getAllPoseObservations().forEach(observation -> {
            fullSwerveOdometry.addVisionMeasurement(
                observation.visionRobotPoseMeters(),
                observation.timestampSeconds(),
                observation.visionMeasurementStdDevs()
            );
        });

        // Report poses
        Pose2d estimatedPosition = new Pose2d(
                onlyWheelsGyroSwerveOdometry.getEstimatedPosition().getTranslation(),
                getCurrentHeading()
        );
        aKitLog.record("OdometryOnlyRobotPose", estimatedPosition);

        Pose2d visionEnhancedPosition = new Pose2d(
                fullSwerveOdometry.getEstimatedPosition().getTranslation(),
                fullSwerveOdometry.getEstimatedPosition().getRotation()
        );
        aKitLog.record("VisionEnhancedPose", visionEnhancedPosition);

        Pose2d robotPose = this.useVisionAssistedPose.get() ? visionEnhancedPosition : estimatedPosition;
        aKitLog.record("RobotPose", robotPose);

        totalDistanceX = robotPose.getX();
        totalDistanceY = robotPose.getY();

        double prevTotalDistanceX = totalDistanceX;
        double prevTotalDistanceY = totalDistanceY;
        this.velocityX = ((totalDistanceX - prevTotalDistanceX));
        this.velocityY = ((totalDistanceY - prevTotalDistanceY));
        this.totalVelocity = (Math.sqrt(Math.pow(velocityX, 2.0) + Math.pow(velocityY, 2.0))); // Unnecessary?
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        // if we have simulated data, return that directly instead of asking the
        // modules
        if(simulatedModulePositions.isPresent()) {
            return simulatedModulePositions.get();
        }
        return new SwerveModulePosition[] {
                drive.getFrontLeftSwerveModuleSubsystem().getCurrentPosition(),
                drive.getFrontRightSwerveModuleSubsystem().getCurrentPosition(),
                drive.getRearLeftSwerveModuleSubsystem().getCurrentPosition(),
                drive.getRearRightSwerveModuleSubsystem().getCurrentPosition()
        };
    }

    public void setCurrentPosition(double newXPositionMeters, double newYPositionMeters, WrappedRotation2d heading) {
        super.setCurrentPosition(newXPositionMeters, newYPositionMeters);
        super.setCurrentHeading(heading.getDegrees());
        onlyWheelsGyroSwerveOdometry.resetPosition(
                heading,
                getSwerveModulePositions(),
                new Pose2d(
                        newXPositionMeters,
                        newYPositionMeters,
                        this.getCurrentHeading()));
        fullSwerveOdometry.resetPosition(
                heading,
                getSwerveModulePositions(),
                new Pose2d(
                        newXPositionMeters,
                        newYPositionMeters,
                        this.getCurrentHeading()));
    }

    public void setCurrentPoseInMeters(Pose2d newPoseInMeters) {
        setCurrentPosition(
                newPoseInMeters.getTranslation().getX(),
                newPoseInMeters.getTranslation().getY(),
                WrappedRotation2d.fromRotation2d(newPoseInMeters.getRotation())
        );
    }

    @Override
    public Pose2d getCurrentPose2d() {
        return useVisionAssistedPose.get() ? new Pose2d(
                fullSwerveOdometry.getEstimatedPosition().getTranslation(),
                fullSwerveOdometry.getEstimatedPosition().getRotation()
        ) : new Pose2d(
                onlyWheelsGyroSwerveOdometry.getEstimatedPosition().getTranslation(),
                onlyWheelsGyroSwerveOdometry.getEstimatedPosition().getRotation()
        );
    }

    // used by the physics simulator to mock what the swerve modules are doing currently for pose estimation
    public void ingestSimulatedSwerveModulePositions(SwerveModulePosition[] positions) {
        this.simulatedModulePositions = Optional.of(positions);
    }

    public Pose2d getClosestReefFacePose() {
        Pose2d currentPose = getCurrentPose2d();

        double closeDistance = convertBlueToRedIfNeeded(
                Landmarks.BlueCloseAlgae).getTranslation().getDistance(currentPose.getTranslation());
        double closeLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                Landmarks.BlueCloseLeftAlgae).getTranslation().getDistance(currentPose.getTranslation());
        double closeRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                Landmarks.BlueCloseRightAlgae).getTranslation().getDistance(currentPose.getTranslation());
        double farLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                Landmarks.BlueFarLeftAlgae).getTranslation().getDistance(currentPose.getTranslation());
        double farDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                Landmarks.BlueFarAlgae).getTranslation().getDistance(currentPose.getTranslation());
        double farRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                Landmarks.BlueFarRightAlgae).getTranslation().getDistance(currentPose.getTranslation());

        HashMap<Double, Pose2d> hashMap = new HashMap<>();
        hashMap.put(closeLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCloseLeftAlgae));
        hashMap.put(closeDistance, PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCloseAlgae));
        hashMap.put(closeRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCloseRightAlgae));
        hashMap.put(farLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueFarLeftAlgae));
        hashMap.put(farDistance, PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueFarAlgae));
        hashMap.put(farRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueFarRightAlgae));

        double leastDistance = closeLeftDistance;

        for (Double distance : hashMap.keySet()) {
            if (distance < leastDistance) {
                leastDistance = distance;
            }
        }
        return hashMap.get(leastDistance);
    }
}
