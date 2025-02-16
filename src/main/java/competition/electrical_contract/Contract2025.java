package competition.electrical_contract;

import java.util.EnumSet;

import javax.inject.Inject;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.injection.electrical_contract.MotorControllerType;
import xbot.common.injection.swerve.SwerveInstance;
import xbot.common.math.XYPair;
import xbot.common.subsystems.vision.CameraCapabilities;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

public class Contract2025 extends ElectricalContract {

    protected final double simulationScalingValue = 256.0 * PoseSubsystem.INCHES_IN_A_METER;

    @Inject
    public Contract2025() {}

    @Override
    public boolean isDriveReady() {
        return true;
    }

    public boolean isAlgaeCollectionReady() { return true; }

    //TODO: change id
    public CANMotorControllerInfo getAlgaeCollectionMotor() {
        return new CANMotorControllerInfo("AlgaeCollectionMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                32,
                new CANMotorControllerOutputConfig());
    }

    public boolean isCoralCollectionMotorReady() { return true; }

    public CANMotorControllerInfo getCoralCollectionMotor() {
        return new CANMotorControllerInfo("CoralCollectionMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                25,
                new CANMotorControllerOutputConfig()
                        .withStatorCurrentLimit(Amps.of(20))
                        .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake)
        );
    }

    public boolean isCoralArmMotorReady() { return true; }

    public CANMotorControllerInfo getCoralArmPivotMotor() {
        return new CANMotorControllerInfo("ArmPivotMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                24,
                new CANMotorControllerOutputConfig().withStatorCurrentLimit(Amps.of(20)));
    }

    public boolean isCoralSensorReady() { return false; }

    @Override
    public DeviceInfo getCoralSensor() {
        return new DeviceInfo("CoralSensor", 0);
    }

    public boolean isElevatorBottomSensorReady() { return false; }

    @Override
    public DeviceInfo getElevatorBottomSensor() { return new DeviceInfo("ElevatorBottomSensor",1); }

    @Override
    public boolean isAlgaeArmPivotMotorReady() {return true;}

    public boolean isAlgaeArmBottomSensorReady(){return false;}

    public DeviceInfo getAlgaeArmBottomSensor() {return new DeviceInfo("AlgaeArmBottomSensor",2); }

    @Override
    public boolean isHumanLoadRampReady() {
        return false;
    }

    @Override
    public CANMotorControllerInfo getAlgaeArmPivotMotor() {
        return new CANMotorControllerInfo("AlgaeArmPivotMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                33,
                new CANMotorControllerOutputConfig()
                        .withStatorCurrentLimit(Amps.of(20))
                        .withInversionType(CANMotorControllerOutputConfig.InversionType.Inverted)
                        .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake)
        );
    }

    @Override
    public boolean areCanCodersReady() {
        return true;
    }
    // change channels
    public DeviceInfo getCoralArmPivotAbsoluteEncoder() {
        return new DeviceInfo("ArmPivotAbsoluteEncoder", 100);
    }

    public boolean isCoralArmPivotAbsoluteEncoderReady() { return false; }

    public DeviceInfo getCoralArmPivotLowSensor() {
        return new DeviceInfo("ArmPivotLowSensor", 101);
    }
    public boolean isCoralArmPivotLowSensorReady() { return false; }

    @Override
    public boolean isElevatorReady() {
        return true; //return true when ready
    }

    @Override
    public CANMotorControllerInfo getElevatorMotor() {

        CANMotorControllerOutputConfig elevatorMotorConfig = new CANMotorControllerOutputConfig()
                .withStatorCurrentLimit(Amps.of(60))
                .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake)
                .withInversionType(CANMotorControllerOutputConfig.InversionType.Inverted);

        return new CANMotorControllerInfo(
                "ElevatorMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO, 23, //change deviceId later
                elevatorMotorConfig);
    }

    @Override
    public boolean isElevatorDistanceSensorReady() {
        return false;
    }

    @Override
    public DeviceInfo getElevatorDistanceSensor() {
        return new DeviceInfo("ElevatorDistanceSensor", 5);
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

    CANMotorControllerOutputConfig regularDriveMotorConfig =
            new CANMotorControllerOutputConfig()
                    .withInversionType(CANMotorControllerOutputConfig.InversionType.Inverted)
                    .withStatorCurrentLimit(Amps.of(45))
                    .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake);

    CANMotorControllerOutputConfig invertedDriveMotorConfig =
            new CANMotorControllerOutputConfig()
                    .withInversionType(CANMotorControllerOutputConfig.InversionType.Inverted)
                    .withStatorCurrentLimit(Amps.of(45))
                    .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake);

    @Override
    public CANMotorControllerInfo getDriveMotor(SwerveInstance swerveInstance) {
                return switch (swerveInstance.label()) {
            case "RearRightDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            39,
                            regularDriveMotorConfig);
            case "FrontLeftDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            31,
                            invertedDriveMotorConfig);
            case "RearLeftDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            20,
                            regularDriveMotorConfig);
            case "FrontRightDrive" ->
                    new CANMotorControllerInfo(
                            getDriveControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            29,
                            invertedDriveMotorConfig);
            default -> null;
        };
    }

    @Override
    public CANMotorControllerInfo getSteeringMotor(SwerveInstance swerveInstance) {

        CANMotorControllerOutputConfig invertedSteeringMotorConfig =
                new CANMotorControllerOutputConfig()
                        .withInversionType(CANMotorControllerOutputConfig.InversionType.Inverted)
                        .withStatorCurrentLimit(Amps.of(45))
                        .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake);

        return switch (swerveInstance.label()) {
            case "RearRightDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            38,
                            invertedSteeringMotorConfig);
            case "FrontLeftDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            30,
                            invertedSteeringMotorConfig);
            case "RearLeftDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            21,
                            invertedSteeringMotorConfig);
            case "FrontRightDrive" ->
                    new CANMotorControllerInfo(
                            getSteeringControllerName(swerveInstance),
                            MotorControllerType.TalonFx,
                            CANBusId.DefaultCanivore,
                            28,
                            invertedSteeringMotorConfig);
            default -> null;
        };
    }

    @Override
    public DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance) {
        double simulationScalingValue = 1.0;

        return switch (swerveInstance.label()) {
            case "RearRightDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), CANBusId.DefaultCanivore, 54, false);
            case "FrontLeftDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), CANBusId.DefaultCanivore, 53, false);
            case "RearLeftDrive" ->
                    new DeviceInfo(getSteeringEncoderControllerName(swerveInstance), CANBusId.DefaultCanivore,52, false);
            case "FrontRightDrive" ->
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

    private static double frontAprilCameraXDisplacement = 10.25 / PoseSubsystem.INCHES_IN_A_METER;
    private static double frontAprilCameraYDisplacement = 6.5 / PoseSubsystem.INCHES_IN_A_METER;
    private static double frontAprilCameraZDisplacement = 7 / PoseSubsystem.INCHES_IN_A_METER;
    private static double frontAprilCameraPitch = Math.toRadians(-12);
    private static double frontAprilCameraYaw = Math.toRadians(0);

    public CameraInfo[] getCameraInfo() {
        return new CameraInfo[] {};
        /*
                new CameraInfo("Apriltag_FrontLeft_Camera",
                        "AprilTagFrontLeft",
                        new Transform3d(new Translation3d(
                                frontAprilCameraXDisplacement,
                                frontAprilCameraYDisplacement,
                                frontAprilCameraZDisplacement),
                                new Rotation3d(0, frontAprilCameraPitch, frontAprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG)),

                new CameraInfo("Apriltag_FrontRight_Camera",
                        "AprilTagFrontRight",
                        new Transform3d(new Translation3d(
                                frontAprilCameraXDisplacement,
                                -frontAprilCameraYDisplacement,
                                frontAprilCameraZDisplacement),
                                new Rotation3d(0, frontAprilCameraPitch, frontAprilCameraYaw)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG)),

                new CameraInfo("Apriltag_Back_Camera",
                        "AprilTagBack",
                        new Transform3d(new Translation3d(
                                -frontAprilCameraXDisplacement,
                                0,
                                frontAprilCameraZDisplacement),
                                new Rotation3d(0, Math.toRadians(-45), Math.PI)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG))
        };
        */
    }

    @Override
    public Distance getDistanceFromCenterToOuterBumperX() {
        return Inches.of(18);
    }
}
