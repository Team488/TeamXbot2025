package competition.electrical_contract;

import java.util.EnumSet;

import javax.inject.Inject;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

import competition.subsystems.pose.PoseSubsystem;
import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.injection.electrical_contract.MotorControllerType;
import xbot.common.injection.swerve.SwerveInstance;
import xbot.common.math.XYPair;
import xbot.common.subsystems.vision.CameraCapabilities;

public class Contract2025 extends ElectricalContract {

    protected final double simulationScalingValue = 256.0 * PoseSubsystem.INCHES_IN_A_METER;

    @Inject
    public Contract2025() {}

    @Override
    public boolean isDriveReady() {
        return true;
    }

    public boolean isAlgaeCollectionReady() { return false; }

    //TODO: change id
    public CANMotorControllerInfo getAlgaeCollectionMotor() {
        return new CANMotorControllerInfo("AlgaeCollectionMotor",
                MotorControllerType.TalonFx,
                CANBusId.DefaultCanivore,
                488,
                new CANMotorControllerOutputConfig());
    }

    public boolean isCoralCollectionMotorReady() { return false; }

    public CANMotorControllerInfo getCoralCollectionMotor(){
        return new CANMotorControllerInfo("CoralCollectionMotor",
                MotorControllerType.TalonFx,
                CANBusId.DefaultCanivore,
                4888,
                new CANMotorControllerOutputConfig());
    }


    @Override
    public boolean areCanCodersReady() {
        return true;
    }

    @Override
    public boolean isElevatorReady() {
        return true; //return true when ready
    }

    @Override
    public CANMotorControllerInfo getElevatorMotor() {
        return new CANMotorControllerInfo(
                "ElevatorMotor",
                MotorControllerType.TalonFx,
                CANBusId.DefaultCanivore, 99, //change deviceId later
                new CANMotorControllerOutputConfig());
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

    @Override
    public CANMotorControllerInfo getDriveMotor(SwerveInstance swerveInstance) {

        CANMotorControllerOutputConfig invertedConfig =
                new CANMotorControllerOutputConfig().withInversionType(
                        CANMotorControllerOutputConfig.InversionType.Inverted);

        return switch (swerveInstance.label()) {
            case "FrontLeftDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            39,
                            new CANMotorControllerOutputConfig());
            case "FrontRightDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            31,
                            invertedConfig);
            case "RearLeftDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            20,
                            new CANMotorControllerOutputConfig());
            case "RearRightDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            29,
                            invertedConfig);
            default -> null;
        };
    }

    @Override
    public CANMotorControllerInfo getSteeringMotor(SwerveInstance swerveInstance) {
        double simulationScalingValue = 1.0;

        CANMotorControllerOutputConfig invertedConfig =
                new CANMotorControllerOutputConfig().withInversionType(
                        CANMotorControllerOutputConfig.InversionType.Inverted);

        return switch (swerveInstance.label()) {
            case "FrontLeftDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            38,
                            invertedConfig);
            case "FrontRightDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            30,
                            invertedConfig);
            case "RearLeftDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            21,
                            invertedConfig);
            case "RearRightDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            28,
                            invertedConfig);
            default -> null;
        };
    }

    @Override
    public DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance) {
        double simulationScalingValue = 1.0;

        return switch (swerveInstance.label()) {
            case "FrontLeftDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), CANBusId.DefaultCanivore, 54, false);
            case "FrontRightDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), CANBusId.DefaultCanivore, 53, false);
            case "RearLeftDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), CANBusId.DefaultCanivore,52, false);
            case "RearRightDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), CANBusId.DefaultCanivore,51, false);
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
    private static double aprilCameraPitch = Math.toRadians(0);
    private static double aprilCameraYaw = Math.toRadians(10);

    public CameraInfo[] getCameraInfo() {
        return new CameraInfo[] {
                new CameraInfo("Apriltag_FrontLeft_Camera",
                        "AprilTagFrontLeft",
                        new Transform3d(new Translation3d(
                                aprilCameraXDisplacement,
                                aprilCameraYDisplacement,
                                aprilCameraZDisplacement),
                                new Rotation3d(0, aprilCameraPitch, aprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG))
        };
    }
}
