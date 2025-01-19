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

import java.util.EnumSet;

import javax.inject.Inject;

public class Contract2023 extends Contract2024 {

    @Inject
    public Contract2023() {}

    @Override
    public boolean isDriveReady() {
        return true;
    }

    @Override
    public boolean areCanCodersReady() {
        return true;
    }

    @Override
    public CANMotorControllerInfo getDriveMotor(SwerveInstance swerveInstance) {
        return switch (swerveInstance.label()) {
            case "FrontLeftDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.SparkMax,
                            CANBusId.RIO,
                            31,
                            new CANMotorControllerOutputConfig());
            case "FrontRightDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.SparkMax,
                            CANBusId.RIO,
                            29,
                            new CANMotorControllerOutputConfig());
            case "RearLeftDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.SparkMax,
                            CANBusId.RIO,
                            38,
                            new CANMotorControllerOutputConfig());
            case "RearRightDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.SparkMax,
                            CANBusId.RIO,
                            21,
                            new CANMotorControllerOutputConfig());
            default -> null;
        };
    }

    @Override
    public CANMotorControllerInfo getSteeringMotor(SwerveInstance swerveInstance) {
        double simulationScalingValue = 1.0;

        return switch (swerveInstance.label()) {
            case "FrontLeftDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.SparkMax,
                            CANBusId.RIO,
                            30,
                            new CANMotorControllerOutputConfig());
            case "FrontRightDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.SparkMax,
                            CANBusId.RIO,
                            28,
                            new CANMotorControllerOutputConfig());
            case "RearLeftDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.SparkMax,
                            CANBusId.RIO,
                            39,
                            new CANMotorControllerOutputConfig());
            case "RearRightDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.SparkMax,
                            CANBusId.RIO,
                            20,
                            new CANMotorControllerOutputConfig());
            default -> null;
        };
    }

    @Override
    public DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance) {
        double simulationScalingValue = 1.0;

        return switch (swerveInstance.label()) {
            case "FrontLeftDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), 51, false, simulationScalingValue);
            case "FrontRightDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), 52, false, simulationScalingValue);
            case "RearLeftDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), 53, false, simulationScalingValue);
            case "RearRightDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), 54, false, simulationScalingValue);
            default -> null;
        };
    }

    @Override
    public XYPair getSwerveModuleOffsetsInInches(SwerveInstance swerveInstance) {
        // Update these XYPairs with the swerve module locations!!! (In inches)
        return switch (swerveInstance.label()) {
            case "FrontLeftDrive" -> new XYPair(15, 15);
            case "FrontRightDrive" -> new XYPair(15, -15);
            case "RearLeftDrive" -> new XYPair(-15, 15);
            case "RearRightDrive" -> new XYPair(-15, -15);
            default -> new XYPair(0, 0);
        };
    }


    private static double aprilCameraXDisplacement = 13.153 / PoseSubsystem.INCHES_IN_A_METER;
    private static double aprilCameraYDisplacement = 12.972 / PoseSubsystem.INCHES_IN_A_METER;
    private static double aprilCameraZDisplacement = 9.014 / PoseSubsystem.INCHES_IN_A_METER;
    private static double aprilCameraPitch = Math.toRadians(-55.5);
    private static double aprilCameraYaw = Math.toRadians(10);

    @Override
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
                                new Rotation3d(0, aprilCameraPitch, Math.toRadians(180) - aprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG)),
                new CameraInfo("Apriltag_RearRight_Camera",
                        "AprilTagRearRight",
                        new Transform3d(new Translation3d(
                                -aprilCameraXDisplacement,
                                -aprilCameraYDisplacement,
                                aprilCameraZDisplacement),
                                new Rotation3d(0, aprilCameraPitch, Math.toRadians(180) + aprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG))
        };
    }
}
