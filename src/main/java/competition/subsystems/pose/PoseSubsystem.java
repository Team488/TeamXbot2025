package competition.subsystems.pose;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.Utilities.Entities.BatchedPushRequests;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.sensors.XGyro.XGyroFactory;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.WrappedRotation2d;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.pose.BasePoseSubsystem;

@Singleton
public class PoseSubsystem extends BasePoseSubsystem {

    final SwerveDrivePoseEstimator onlyWheelsGyroSwerveOdometry;
    final SwerveDrivePoseEstimator fullSwerveOdometry;

    private final DriveSubsystem drive;
    private final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    private final BooleanProperty useVisionAssistedPose;
    private final BooleanProperty reportCameraPoses;
    private final CoprocessorCommunicationSubsystem coprocessorComms;

    public static final Distance fieldXMidpointInMeters = Meters.of(8.7785);
    public static final Distance fieldYMidpointInMeters = Meters.of(4.025);

    private boolean preferOdometryToVision = false;


    // only used when simulating the robot
    protected Optional<SwerveModulePosition[]> simulatedModulePositions = Optional.empty();

    @Inject
    public PoseSubsystem(XGyroFactory gyroFactory, PropertyFactory propManager, DriveSubsystem drive,
                         AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, CoprocessorCommunicationSubsystem coprocessorComms) {
        super(gyroFactory, propManager);
        this.drive = drive;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.coprocessorComms = coprocessorComms;

        onlyWheelsGyroSwerveOdometry = initializeSwerveOdometry();
        fullSwerveOdometry = initializeSwerveOdometry();

        propManager.setPrefix(this);
        propManager.setDefaultLevel(Property.PropertyLevel.Important);
        useVisionAssistedPose = propManager.createPersistentProperty("UseVisionAssistedPose", true);
        reportCameraPoses = propManager.createPersistentProperty("ReportCameraPoses", false);
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
                getCurrentHeadingGyroOnly(),
                getSwerveModulePositions(),
                new Pose2d());
    }

    @Override
    protected void updateOdometry() {

        String xtablesPrefix = "PoseSubsystem";
        // Package all requests into single message to ensure all data is synchronized and updated at once.
        BatchedPushRequests batchedPushRequests = new BatchedPushRequests();

        // Update pose estimators
        onlyWheelsGyroSwerveOdometry.update(
                this.getCurrentHeadingGyroOnly(),
                getSwerveModulePositions()
        );
        aKitLog.record("WheelsOnlyEstimate", onlyWheelsGyroSwerveOdometry.getEstimatedPosition());


        batchedPushRequests.putPose2d(xtablesPrefix + ".WheelsOnlyEstimate", onlyWheelsGyroSwerveOdometry.getEstimatedPosition());
        fullSwerveOdometry.update(
                this.getCurrentHeadingGyroOnly(),
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
        Pose2d odometryOnlyRobotPose = new Pose2d(
                onlyWheelsGyroSwerveOdometry.getEstimatedPosition().getTranslation(),
                getCurrentHeadingGyroOnly()
        );

        aKitLog.record("OdometryOnlyRobotPose", odometryOnlyRobotPose);
        batchedPushRequests.putPose2d(xtablesPrefix + ".OdometryOnlyRobotPose", odometryOnlyRobotPose);

        Pose2d visionEnhancedPosition = new Pose2d(
                fullSwerveOdometry.getEstimatedPosition().getTranslation(),
                fullSwerveOdometry.getEstimatedPosition().getRotation()
        );
        aKitLog.record("VisionEnhancedPose", visionEnhancedPosition);
        batchedPushRequests.putPose2d(xtablesPrefix + ".VisionEnhancedPose", visionEnhancedPosition);

        Pose2d robotPose = odometryOnlyRobotPose;
        if (this.useVisionAssistedPose.get() && !preferOdometryToVision) {
            robotPose = visionEnhancedPosition;
        }

        aKitLog.record("RobotPose", robotPose);
        batchedPushRequests.putPose2d(xtablesPrefix + ".RobotPose", robotPose);
        XTablesClient xTablesClient = this.coprocessorComms.getXTablesManager().getOrNull();
        if (xTablesClient != null) {
            // This is asynchronous - does not block & sends all updates in a single "packet"
            xTablesClient.sendBatchedPushRequests(batchedPushRequests);
        }

        // Record the camera positions
        if (reportCameraPoses.get()) {
            var robotPose3d = new Pose3d(
                    robotPose.getTranslation().getX(),
                    robotPose.getTranslation().getY(),
                    -0.5, // Reverse offset in Advantage Scope
                    new Rotation3d(robotPose.getRotation()));
            for (int i = 0; i < aprilTagVisionSubsystem.getCameraCount(); i++) {
                var cameraPosition = aprilTagVisionSubsystem.getCameraPosition(i);
                aKitLog.record("CameraPose/" + i, robotPose3d.transformBy(cameraPosition));
            }
        }

        totalDistanceX = robotPose.getX();
        totalDistanceY = robotPose.getY();

        this.velocityX = ((totalDistanceX - prevTotalDistanceX));
        this.velocityY = ((totalDistanceY - prevTotalDistanceY));
        double deltaTime = XTimer.getFPGATimestamp() - previousTimestamp;
        if (deltaTime < 0.0001) {
            // protecting against future divide by zero
            deltaTime = 0.0001;
        }

        prevTotalDistanceX = totalDistanceX;
        prevTotalDistanceY = totalDistanceY;
        previousTimestamp = XTimer.getFPGATimestamp();

        aKitLog.setLogLevel(AKitLogger.LogLevel.INFO);

        var motionThisTick = Math.sqrt(Math.pow(velocityX, 2.0) + Math.pow(velocityY, 2.0));

        this.totalVelocity = motionThisTick / deltaTime;
        aKitLog.record("TotalVelocity", this.totalVelocity);
    }

    double prevTotalDistanceX;
    double prevTotalDistanceY;
    double previousTimestamp = 0;

    public double getAbsoluteVelocity() {
        return this.totalVelocity;
    }


    /**
     * Get a command that resets the pose estimator to the current vision estimate
     * @return The command that resets the pose estimator
     */
    public Command getResetTranslationToVisionEstimateCommand() {
        return new InstantCommand(() -> {
            var estimatedPose = new Pose2d(
                    fullSwerveOdometry.getEstimatedPosition().getTranslation(),
                    getCurrentHeadingGyroOnly());
            resetPoseEstimator(estimatedPose);
        }).ignoringDisable(true);
    }

    /**
     * Get a command that resets the pose estimator to a specific pose
     * @param pose The pose to reset the estimator to
     * @return The command that resets the pose estimator
     */
    public Command getResetPoseCommand(Pose2d pose) {
        return new InstantCommand(() -> resetPoseEstimator(pose))
                .ignoringDisable(true);
    }

    private void resetPoseEstimator(Pose2d pose) {
        this.fullSwerveOdometry.resetPose(pose);
        this.onlyWheelsGyroSwerveOdometry.resetPose(pose);
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        // if we have simulated data, return that directly instead of asking the
        // modules
        if (simulatedModulePositions.isPresent()) {
            return simulatedModulePositions.get();
        }
        return new SwerveModulePosition[]{
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
                        this.getCurrentHeadingGyroOnly()));
        fullSwerveOdometry.resetPosition(
                heading,
                getSwerveModulePositions(),
                new Pose2d(
                        newXPositionMeters,
                        newYPositionMeters,
                        this.getCurrentHeadingGyroOnly()));
    }

    public void setCurrentPosition(Pose2d pose) {
        setCurrentPosition(pose.getTranslation().getX(), pose.getTranslation().getY(), WrappedRotation2d.fromRotation2d(pose.getRotation()));
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

    @Override
    public WrappedRotation2d getCurrentHeading() {
        if (useVisionAssistedPose.get()) {
            return WrappedRotation2d.fromRotation2d(fullSwerveOdometry.getEstimatedPosition().getRotation());
        } else {
            return WrappedRotation2d.fromRotation2d(onlyWheelsGyroSwerveOdometry.getEstimatedPosition().getRotation());
        }
    }

    // used by the physics simulator to mock what the swerve modules are doing currently for pose estimation
    public void ingestSimulatedSwerveModulePositions(SwerveModulePosition[] positions) {
        this.simulatedModulePositions = Optional.of(positions);
    }

    public Pose2d getClosestReefFacePose() {
        Pose2d currentPose = getCurrentPose2d();

        List<Pose2d> reefFacePoses = Arrays.asList(
                convertBlueToRedIfNeeded(Landmarks.BlueCloseAlgae),
                convertBlueToRedIfNeeded(Landmarks.BlueCloseLeftAlgae),
                convertBlueToRedIfNeeded(Landmarks.BlueCloseRightAlgae),
                convertBlueToRedIfNeeded(Landmarks.BlueFarLeftAlgae),
                convertBlueToRedIfNeeded(Landmarks.BlueFarAlgae),
                convertBlueToRedIfNeeded(Landmarks.BlueFarRightAlgae));

        return currentPose.nearest(reefFacePoses);
    }

    public Landmarks.ReefFace getClosestReefFace() {
        Pose2d currentPose = getCurrentPose2d();
        var closeReefFace = convertBlueToRedIfNeeded(Landmarks.BlueCloseAlgae);
        var closeLeftReefFace = convertBlueToRedIfNeeded(Landmarks.BlueCloseLeftAlgae);
        var closeRightReefFace = convertBlueToRedIfNeeded(Landmarks.BlueCloseRightAlgae);
        var farReefFace = convertBlueToRedIfNeeded(Landmarks.BlueFarAlgae);
        var farLeftReefFace = convertBlueToRedIfNeeded(Landmarks.BlueFarLeftAlgae);
        var farRightReefFace = convertBlueToRedIfNeeded(Landmarks.BlueFarRightAlgae);

        List<Pose2d> reefFacePoses = Arrays.asList(
                closeReefFace, closeLeftReefFace, closeRightReefFace,
                farReefFace, farLeftReefFace, farRightReefFace);
        HashMap<Pose2d, Landmarks.ReefFace> hashMap = new HashMap<>();
        hashMap.put(closeReefFace, Landmarks.ReefFace.CLOSE);
        hashMap.put(closeLeftReefFace, Landmarks.ReefFace.CLOSE_LEFT);
        hashMap.put(closeRightReefFace, Landmarks.ReefFace.CLOSE_RIGHT);
        hashMap.put(farReefFace, Landmarks.ReefFace.FAR);
        hashMap.put(farLeftReefFace, Landmarks.ReefFace.FAR_LEFT);
        hashMap.put(farRightReefFace, Landmarks.ReefFace.FAR_RIGHT);

        return hashMap.get(currentPose.nearest(reefFacePoses));
    }

    public Landmarks.CoralStation getClosestCoralStation() {
        Pose2d currentPose = getCurrentPose2d();
        var leftCoralStation = convertBlueToRedIfNeeded(Landmarks.BlueLeftCoralStationMid);
        var rightCoralStation = convertBlueToRedIfNeeded(Landmarks.BlueRightCoralStationMid);

        List<Pose2d> coralStationPoses = Arrays.asList(leftCoralStation,rightCoralStation);
        HashMap<Pose2d, Landmarks.CoralStation> hashMap = new HashMap<>();
        hashMap.put(leftCoralStation,Landmarks.CoralStation.LEFT);
        hashMap.put(rightCoralStation, Landmarks.CoralStation.RIGHT);

        return hashMap.get(currentPose.nearest(coralStationPoses));
    }

    public Landmarks.ReefFace getReefFaceFromAngle() {
        double currentAngleInDegrees;
        if (useVisionAssistedPose.get()) {
            currentAngleInDegrees = PoseSubsystem.convertBlueToRedIfNeeded(fullSwerveOdometry.getEstimatedPosition().getRotation()).getDegrees();
        } else {
            currentAngleInDegrees =  PoseSubsystem.convertBlueToRedIfNeeded(onlyWheelsGyroSwerveOdometry.getEstimatedPosition().getRotation()).getDegrees();
        }
        
        if (currentAngleInDegrees > 150 || currentAngleInDegrees < -150) {
            return Landmarks.ReefFace.FAR;
        }
        else if (currentAngleInDegrees > 90) {
            return Landmarks.ReefFace.FAR_RIGHT;
        }
        else if (currentAngleInDegrees > 30) {
            return Landmarks.ReefFace.CLOSE_RIGHT;
        }
        else if (currentAngleInDegrees > -30) {
            return Landmarks.ReefFace.CLOSE;
        }
        else if (currentAngleInDegrees > -90) {
            return Landmarks.ReefFace.CLOSE_LEFT;
        }
        else {
            return Landmarks.ReefFace.FAR_LEFT;
        }
    }

    public Command createSetPositionCommand(Pose2d pose) {
        return Commands.runOnce(() -> setCurrentPosition(pose));
    }

    public Command createSetPositionCommand(Supplier<Pose2d> poseSupplier) {
        return Commands.runOnce(() -> setCurrentPosition(poseSupplier.get())).ignoringDisable(true);
    }

    public Command createSetPositionCommandThatMirrorsIfNeeded(Pose2d bluePose) {
        return Commands.runOnce(() -> setCurrentPosition(PoseSubsystem.convertBlueToRedIfNeeded(bluePose))).ignoringDisable(true);
    }

    public void setPreferOdometryToVision(boolean preferOdometryToVision) {
        this.preferOdometryToVision = preferOdometryToVision;
        if (preferOdometryToVision) {
            // If we are disabling vision updates, we need to "snap" the odometry estimate to the vision estimate.
            // This is because we will be using the odometry estimate while vision is being supresse, and we need
            // to avoid any callers of the PoseSubsystem experiencing discontinuities.
            resetPoseEstimator(fullSwerveOdometry.getEstimatedPosition());
        }
    }

    public DriverRelativeCameraValues getDriverRelativeCameraToUse(boolean hasCameraFlippedDriverRelative, int cameraToUse) {
        List<Integer> farReefFacePoseIDList = Arrays.asList(20, 21, 22, 9, 10 , 11);
        List<Integer> closeReefFacePoseIDList = Arrays.asList(19, 18, 17, 6, 7, 8);

        // if our target april tag is a far april tag and cameras haven't been flipped,
        // flip and use the other front camera to align with tag
        if (farReefFacePoseIDList.contains(aprilTagVisionSubsystem.getTargetAprilTagID(getClosestReefFacePose()))
                && !hasCameraFlippedDriverRelative) {
            cameraToUse = (cameraToUse + 1) % 2;
            hasCameraFlippedDriverRelative = true;
        }
        // if our target april tag is a close april tag and cameras have been flipped,
        // flip and use the other front camera to align with tag
        else if (closeReefFacePoseIDList.contains(aprilTagVisionSubsystem.getTargetAprilTagID(getClosestReefFacePose()))
                && hasCameraFlippedDriverRelative) {
            cameraToUse = (cameraToUse + 1) % 2;
            hasCameraFlippedDriverRelative = false;
        }
        return new DriverRelativeCameraValues(hasCameraFlippedDriverRelative, cameraToUse);
    }
}