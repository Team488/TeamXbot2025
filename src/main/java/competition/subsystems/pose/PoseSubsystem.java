package competition.subsystems.pose;

import java.util.Optional;
import javax.inject.Inject;
import javax.inject.Singleton;
import competition.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import xbot.common.controls.sensors.XGyro.XGyroFactory;
import xbot.common.math.WrappedRotation2d;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.pose.BasePoseSubsystem;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;
import competition.subsystems.deadwheel.DeadwheelSubsystem;

@Singleton
public class PoseSubsystem extends BasePoseSubsystem {

    final SwerveDrivePoseEstimator onlyWheelsGyroSwerveOdometry;
    final SwerveDrivePoseEstimator fullSwerveOdometry;
    final SwerveDrivePoseEstimator onlyDeadwheelOdometry;
    final SwerveDrivePoseEstimator fullVisionDeadwheelOdometry;
    final SwerveDrivePoseEstimator fullSwerveDeadwheelOdometry;

    private final DriveSubsystem drive;
    private final AprilTagVisionSubsystem aprilTagVisionSubsystem;
    private final DeadwheelSubsystem deadwheelSubsystem;

    protected Optional<SwerveModulePosition[]> simulatedModulePositions = Optional.empty();

    @Inject
    public PoseSubsystem(XGyroFactory gyroFactory, PropertyFactory propManager, DriveSubsystem drive, 
                         AprilTagVisionSubsystem aprilTagVisionSubsystem, DeadwheelSubsystem deadwheelSubsystem) {
        super(gyroFactory, propManager);
        this.drive = drive;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.deadwheelSubsystem = deadwheelSubsystem;

        onlyWheelsGyroSwerveOdometry = initializeSwerveOdometry();
        fullSwerveOdometry = initializeSwerveOdometry();
        onlyDeadwheelOdometry = initializeDeadwheelOdometry();
        fullVisionDeadwheelOdometry = initializeDeadwheelOdometry();
        fullSwerveDeadwheelOdometry = initializeDeadwheelOdometry();
    }

    private SwerveDrivePoseEstimator initializeSwerveOdometry() {
        return new SwerveDrivePoseEstimator(
                drive.getSwerveDriveKinematics(),
                getCurrentHeading(),
                getSwerveModulePositions(),
                new Pose2d());
    }

    private SwerveDrivePoseEstimator initializeDeadwheelOdometry() {
        return new SwerveDrivePoseEstimator(
                drive.getSwerveDriveKinematics(),
                getCurrentHeading(),
                getDeadwheelPositions(),
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

        onlyDeadwheelOdometry.update(
                this.getCurrentHeading(),
                getDeadwheelPositions()
        );
        aKitLog.record("DeadwheelOnlyEstimate", onlyDeadwheelOdometry.getEstimatedPosition());

        fullVisionDeadwheelOdometry.update(
                this.getCurrentHeading(),
                getDeadwheelPositions()
        );
        this.aprilTagVisionSubsystem.getAllPoseObservations().forEach(observation -> {
            fullVisionDeadwheelOdometry.addVisionMeasurement(
                observation.visionRobotPoseMeters(),
                observation.timestampSeconds(),
                observation.visionMeasurementStdDevs()
            );
        });
        aKitLog.record("FullVisionDeadwheelEstimate", fullVisionDeadwheelOdometry.getEstimatedPosition());

        fullSwerveDeadwheelOdometry.update(
                this.getCurrentHeading(),
                getSwerveModulePositions()
        );
        fullSwerveDeadwheelOdometry.update(
                this.getCurrentHeading(),
                getDeadwheelPositions()
        );
        this.aprilTagVisionSubsystem.getAllPoseObservations().forEach(observation -> {
            fullSwerveDeadwheelOdometry.addVisionMeasurement(
                observation.visionRobotPoseMeters(),
                observation.timestampSeconds(),
                observation.visionMeasurementStdDevs()
            );
        });
        aKitLog.record("FullSwerveDeadwheelEstimate", fullSwerveDeadwheelOdometry.getEstimatedPosition());

        // Report poses
        Pose2d estimatedPosition = new Pose2d(
                onlyWheelsGyroSwerveOdometry.getEstimatedPosition().getTranslation(),
                getCurrentHeading()
        );
        aKitLog.record("RobotPose", estimatedPosition);

        Pose2d visionEnhancedPosition = new Pose2d(
                fullSwerveOdometry.getEstimatedPosition().getTranslation(),
                fullSwerveOdometry.getEstimatedPosition().getRotation()
        );
        aKitLog.record("VisionEnhancedPose", visionEnhancedPosition);

        totalDistanceX = estimatedPosition.getX();
        totalDistanceY = estimatedPosition.getY();

        double prevTotalDistanceX = totalDistanceX;
        double prevTotalDistanceY = totalDistanceY;
        this.velocityX = ((totalDistanceX - prevTotalDistanceX));
        this.velocityY = ((totalDistanceY - prevTotalDistanceY));
        this.totalVelocity = (Math.sqrt(Math.pow(velocityX, 2.0) + Math.pow(velocityY, 2.0))); // Unnecessary?
    }

    private SwerveModulePosition[] getDeadwheelPositions() {
        return new SwerveModulePosition[] {
                new SwerveModulePosition(deadwheelSubsystem.getLeftDistance(), new WrappedRotation2d(0)),
                new SwerveModulePosition(deadwheelSubsystem.getRightDistance(), new WrappedRotation2d(0)),
                new SwerveModulePosition(deadwheelSubsystem.getFrontDistance(), new WrappedRotation2d(0))
        };
    }

    // Override methods remain unchanged

    @Override
    protected double getLeftDriveDistance() {
        return drive.getLeftTotalDistance();
    }

    @Override
    protected double getRightDriveDistance() {
        return drive.getRightTotalDistance();
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
    }

    public void setCurrentPoseInMeters(Pose2d newPoseInMeters) {
        setCurrentPosition(
                newPoseInMeters.getTranslation().getX(),
                newPoseInMeters.getTranslation().getY(),
                WrappedRotation2d.fromRotation2d(newPoseInMeters.getRotation())
        );
    }

    @Override
    public Pose2d getGroundTruthPose() {
        return this.getCurrentPose2d();
    }

    public void ingestSimulatedSwerveModulePositions(SwerveModulePosition[] positions) {
        this.simulatedModulePositions = Optional.of(positions);
    }
}
