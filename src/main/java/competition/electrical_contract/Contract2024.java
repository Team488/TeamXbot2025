package competition.electrical_contract;

import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.injection.electrical_contract.MotorControllerType;
import xbot.common.injection.swerve.SwerveInstance;
import xbot.common.math.XYPair;
import xbot.common.subsystems.vision.CameraCapabilities;

import javax.inject.Inject;
import java.util.EnumSet;
import java.util.Map;

public class Contract2024 extends Contract2025 {

    @Inject
    public Contract2024() {}

    @Override
    public boolean isDriveReady() {
        return true;
    }

    @Override
    public boolean areCanCodersReady() {
        return true;
    }

    protected String getDriveControllerName(SwerveInstance swerveInstance) {
        return "DriveSubsystem/" + swerveInstance.label() + "/Drive";
    }

    protected String getSteeringControllerName(SwerveInstance swerveInstance) {
        return "DriveSubsystem/" + swerveInstance.label() + "/Steering";
    }

    protected String getSteeringEncoderControllerName(SwerveInstance swerveInstance) {
        return "DriveSubsystem/" + swerveInstance.label() + "/SteeringEncoder";
    }

    // Swerve motor configuration structure
    private static class SwerveModuleConfig {
        final int driveMotorId;
        final int steeringMotorId;
        final int steeringEncoderId;
        final XYPair moduleOffsets; // in inches
        
        SwerveModuleConfig(int driveMotorId, int steeringMotorId, int steeringEncoderId, XYPair moduleOffsets) {
            this.driveMotorId = driveMotorId;
            this.steeringMotorId = steeringMotorId;
            this.steeringEncoderId = steeringEncoderId;
            this.moduleOffsets = moduleOffsets;
        }
    }
    
    // Centralized swerve module configuration map
    private static final Map<String, SwerveModuleConfig> SWERVE_CONFIG = Map.of(
        "FrontLeftDrive",  new SwerveModuleConfig(39, 38, 54, new XYPair(15, 15)),
        "FrontRightDrive", new SwerveModuleConfig(31, 30, 53, new XYPair(15, -15)),
        "RearLeftDrive",   new SwerveModuleConfig(20, 21, 52, new XYPair(-15, 15)),
        "RearRightDrive",  new SwerveModuleConfig(29, 28, 51, new XYPair(-15, -15))
    );

    @Override
    public CANMotorControllerInfo getDriveMotor(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        if (config == null) {
            return null;
        }
        
        return new CANMotorControllerInfo(
                getDriveControllerName(swerveInstance),
                MotorControllerType.SparkMax,
                CANBusId.RIO,
                config.driveMotorId,
                null,
                new CANMotorControllerOutputConfig());
    }

    @Override
    public CANMotorControllerInfo getSteeringMotor(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        if (config == null) {
            return null;
        }
        
        return new CANMotorControllerInfo(
                getSteeringControllerName(swerveInstance),
                MotorControllerType.SparkMax,
                CANBusId.RIO,
                config.steeringMotorId,
                null,
                new CANMotorControllerOutputConfig());
    }

    @Override
    public DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        if (config == null) {
            return null;
        }
        
        double simulationScalingValue = 1.0;
        return new DeviceInfo(
                getSteeringEncoderControllerName(swerveInstance),
                config.steeringEncoderId,
                false,
                simulationScalingValue,
                null);
    }

    @Override
    public XYPair getSwerveModuleOffsetsInInches(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        if (config != null) {
            return config.moduleOffsets;
        }
        return new XYPair(0, 0);
    }

    @Override
    public double getSteeringGearRatio() {
        return 12.8; // Documented for Swerve Specialties MK4
    }

    @Override
    public double getDriveGearRatio() {
        return 6.12; // Documented value for Swerve Specialties MK4 with L3 ratio.
    }

    private static double aprilCameraXDisplacement = 13.153 / PoseSubsystem.INCHES_IN_A_METER;
    private static double aprilCameraYDisplacement = 12.972 / PoseSubsystem.INCHES_IN_A_METER;
    private static double aprilCameraZDisplacement = 9.014 / PoseSubsystem.INCHES_IN_A_METER;
    private static double aprilCameraPitch = Math.toRadians(-55.5);
    private static double aprilCameraYaw = Math.toRadians(10);

    private static double rearAprilCameraPitch = Math.toRadians(0);
    private static double rearAprilCameraYaw = Math.toRadians(180);

    public static String rearLeftNoteCameraName = "NoteRearLeft";
    public static String rearRightNoteCameraName = "NoteRearRight";
    public static String rearCenterNoteCameraName= "NoteRearCenter"; //TODO - one of these cameras

    public CameraInfo[] getCameraInfo() {
        return new CameraInfo[] {
                new CameraInfo("Apriltag_FrontLeft_Camera",
                        "AprilTagFrontLeft",
                        new Transform3d(new Translation3d(
                                aprilCameraXDisplacement,
                                aprilCameraYDisplacement,
                                aprilCameraZDisplacement),
                                new Rotation3d(0, aprilCameraPitch, aprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG)),
                new CameraInfo("Apriltag_FrontRight_Camera",
                        "AprilTagFrontRight",
                        new Transform3d(new Translation3d(
                                aprilCameraXDisplacement,
                                -aprilCameraYDisplacement,
                                aprilCameraZDisplacement),
                                new Rotation3d(0, aprilCameraPitch, -aprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG)),
                new CameraInfo("Apriltag_RearLeft_Camera",
                        "AprilTagRearLeft",
                        new Transform3d(new Translation3d(
                                -aprilCameraXDisplacement,
                                aprilCameraYDisplacement,
                                aprilCameraZDisplacement),
                                new Rotation3d(0, rearAprilCameraPitch, rearAprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG)),
                new CameraInfo("Apriltag_RearRight_Camera",
                        "AprilTagRearRight",
                        new Transform3d(new Translation3d(
                                -aprilCameraXDisplacement,
                                -aprilCameraYDisplacement,
                                aprilCameraZDisplacement),
                                new Rotation3d(0, rearAprilCameraPitch, rearAprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG))
                        /* ,
                new CameraInfo("GamePiece_FrontLeft_Camera",
                        rearCenterNoteCameraName,
                        new Transform3d(new Translation3d(), new Rotation3d()),
                        EnumSet.of(CameraCapabilities.GAME_SPECIFIC)),
                new CameraInfo("GamePiece_FrontRight_Camera",
                        "NoteFrontRight",
                        new Transform3d(new Translation3d(), new Rotation3d()),
                        EnumSet.of(CameraCapabilities.GAME_SPECIFIC)),
                new CameraInfo("GamePiece_RearLeft_Camera",
                        rearLeftNoteCameraName,
                        new Transform3d(new Translation3d(), new Rotation3d()),
                        EnumSet.of(CameraCapabilities.GAME_SPECIFIC)),
                new CameraInfo("GamePiece_RearRight_Camera",
                        rearRightNoteCameraName,
                        new Transform3d(new Translation3d(), new Rotation3d()),
                        EnumSet.of(CameraCapabilities.GAME_SPECIFIC))*/
        };
    }

    @Override
    public boolean isDeadWheelOdometryReady() { return false; }
}
