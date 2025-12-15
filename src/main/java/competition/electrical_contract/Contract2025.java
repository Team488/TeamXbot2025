package competition.electrical_contract;

import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.controls.sensors.XGyro;
import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig;
import xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig.InversionType;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.injection.electrical_contract.IMUInfo;
import xbot.common.injection.electrical_contract.MotorControllerType;
import xbot.common.injection.electrical_contract.PDHPort;
import xbot.common.injection.electrical_contract.PowerSource;
import xbot.common.injection.swerve.SwerveInstance;
import xbot.common.math.XYPair;
import xbot.common.subsystems.vision.CameraCapabilities;

import javax.inject.Inject;
import java.util.EnumSet;
import java.util.Map;
import java.util.HashMap;

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

    public CANMotorControllerInfo getAlgaeCollectionMotor() {
        return new CANMotorControllerInfo("AlgaeCollectionMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                32,
                PDHPort.PDH10,
                new CANMotorControllerOutputConfig().withInversionType(InversionType.Inverted));
    }

    public boolean isCoralCollectionMotorReady() { return true; }

    public CANMotorControllerInfo getCoralCollectionMotor() {
        return new CANMotorControllerInfo("CoralCollectionMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                25,
                PDHPort.PDH11,
                new CANMotorControllerOutputConfig()
                        .withStatorCurrentLimit(Amps.of(28))
                        .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake));
    }

    public boolean isCoralArmMotorReady() { return true; }

    public CANMotorControllerInfo getCoralArmPivotMotor() {
        return new CANMotorControllerInfo("ArmPivotMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                24,
                PDHPort.PDH12,
                new CANMotorControllerOutputConfig().withStatorCurrentLimit(Amps.of(45)));
    }

    public boolean isCoralScorerSensorReady() { return true; }

    @Override
    public DeviceInfo getCoralScorerSensor() {
        return new DeviceInfo("CoralSensor", 0, true, PowerSource.RIO);
    }

    public boolean isElevatorBottomSensorReady() { return true; }

    @Override
    public DeviceInfo getElevatorBottomSensor() { return new DeviceInfo("ElevatorBottomSensor",3, true, PowerSource.RIO); }


    @Override
    public boolean isHumanLoadRampReady() {
        return false;
    }


    @Override
    public boolean areCanCodersReady() {
        return true;
    }

    public DeviceInfo getCoralArmPivotAbsoluteEncoder() {
        return new DeviceInfo("ArmPivotAbsoluteEncoder", 29, PowerSource.VRM1_12V_500MA);
    }

    public boolean isCoralArmPivotAbsoluteEncoderReady() { return false; }

    public DeviceInfo getCoralArmLowSensor() {
        return new DeviceInfo("ArmPivotLowSensor", 1, true, PowerSource.RIO);
    }
    public boolean isCoralArmLowSensorReady() { return true; }

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
                PDHPort.PDH13,
                elevatorMotorConfig);
    }

    @Override
    public boolean isElevatorDistanceSensorReady() {
        return true;
    }

    @Override
    public DeviceInfo getElevatorDistanceSensor() {
        return new DeviceInfo("ElevatorDistanceSensor", CANBusId.RIO, 5, PowerSource.VRM1_12V_2B);
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
        final PDHPort drivePowerPort;
        final PDHPort steeringPowerPort;
        
        SwerveModuleConfig(int driveMotorId, int steeringMotorId, int steeringEncoderId, 
                          XYPair moduleOffsets, PDHPort drivePowerPort, PDHPort steeringPowerPort) {
            this.driveMotorId = driveMotorId;
            this.steeringMotorId = steeringMotorId;
            this.steeringEncoderId = steeringEncoderId;
            this.moduleOffsets = moduleOffsets;
            this.drivePowerPort = drivePowerPort;
            this.steeringPowerPort = steeringPowerPort;
        }
    }
    
    // Centralized swerve module configuration map
    private static final Map<String, SwerveModuleConfig> SWERVE_CONFIG = Map.of(
        "FrontLeftDrive",  new SwerveModuleConfig(29, 28, 51, 
            new XYPair(12, 12), PDHPort.PDH00, PDHPort.PDH01),
        "FrontRightDrive", new SwerveModuleConfig(39, 38, 54, 
            new XYPair(12, -12), PDHPort.PDH02, PDHPort.PDH03),
        "RearLeftDrive",   new SwerveModuleConfig(20, 21, 52, 
            new XYPair(-12, 12), PDHPort.PDH04, PDHPort.PDH05),
        "RearRightDrive",  new SwerveModuleConfig(31, 30, 53, 
            new XYPair(-12, -12), PDHPort.PDH06, PDHPort.PDH07)
    );

    CANMotorControllerOutputConfig regularDriveMotorConfig =
            new CANMotorControllerOutputConfig()
                    .withInversionType(CANMotorControllerOutputConfig.InversionType.Normal)
                    .withStatorCurrentLimit(Amps.of(80))
                    .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake);

    CANMotorControllerOutputConfig invertedSteeringMotorConfig =
            new CANMotorControllerOutputConfig()
                    .withInversionType(CANMotorControllerOutputConfig.InversionType.Inverted)
                    .withStatorCurrentLimit(Amps.of(45))
                    .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake);

    @Override
    public CANMotorControllerInfo getDriveMotor(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        if (config == null) {
            return null;
        }
        
        return new CANMotorControllerInfo(
                getDriveControllerName(swerveInstance),
                MotorControllerType.TalonFx,
                CANBusId.Canivore,
                config.driveMotorId,
                config.drivePowerPort,
                regularDriveMotorConfig);
    }

    @Override
    public CANMotorControllerInfo getSteeringMotor(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        if (config == null) {
            return null;
        }
        
        return new CANMotorControllerInfo(
                getSteeringControllerName(swerveInstance),
                MotorControllerType.TalonFx,
                CANBusId.Canivore,
                config.steeringMotorId,
                config.steeringPowerPort,
                invertedSteeringMotorConfig);
    }

    @Override
    public DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        if (config == null) {
            return null;
        }
        
        return new DeviceInfo(
                getSteeringEncoderControllerName(swerveInstance),
                CANBusId.Canivore,
                config.steeringEncoderId,
                false,
                PowerSource.MOTOR); 
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
    public IMUInfo getNavXGyroInfo() {
        return new IMUInfo("IMU", XGyro.ImuType.navX, XGyro.InterfaceType.spi, 
        null, 1, PowerSource.RIO);
    }

    @Override
    public IMUInfo getPigeon2GyroInfo() {
        return new IMUInfo("IMU_Pigeon2", XGyro.ImuType.pigeon2, XGyro.InterfaceType.CAN, 
        CANBusId.Canivore, 10, PowerSource.VRM1_12V_2B);
    }

    @Override
    public double getSteeringGearRatio() {
        return 12.1; // Documented value for WCP x2i.
    }

    @Override
    public double getDriveGearRatio() {
        return 6.48; // Documented value for WCP x2i with X3 10t gears.
    }

    private static double frontAprilCameraXDisplacement = 10.14 / PoseSubsystem.INCHES_IN_A_METER;
    private static double frontAprilCameraYDisplacement = 6.535 / PoseSubsystem.INCHES_IN_A_METER;
    private static double frontAprilCameraZDisplacement = 6.7 / PoseSubsystem.INCHES_IN_A_METER;
    private static double frontAprilCameraPitch = Math.toRadians(-21);
    private static double frontAprilCameraYaw = Math.toRadians(0);

    public CameraInfo[] getCameraInfo() {
        return new CameraInfo[] {// {};

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
                                -0.55 / PoseSubsystem.INCHES_IN_A_METER,
                                -0.25 / PoseSubsystem.INCHES_IN_A_METER,
                                6.3 / PoseSubsystem.INCHES_IN_A_METER),
                                new Rotation3d(0, Math.toRadians(-14.5), Math.PI)),
                        EnumSet.of(CameraCapabilities.APRIL_TAG),
                        false)
        };
    }

    @Override
    public Distance getDistanceFromCenterToOuterBumperX() {
        return Inches.of(18);
    }

    @Override
    public DeviceInfo getLightsDio0() {
        return new DeviceInfo("Lights0", 10, PowerSource.RIO);
    }

    @Override
    public DeviceInfo getLightsDio1() {
        return new DeviceInfo("Lights1", 11, PowerSource.RIO);
    }

    @Override
    public DeviceInfo getLightsDio2() {
        return new DeviceInfo("Lights2", 12, PowerSource.RIO);
    }

    @Override
    public DeviceInfo getLightsDio3() {
        return new DeviceInfo("Lights3", 13, PowerSource.RIO);
    }

    @Override
    public boolean isDeadWheelOdometryReady() { return true; }

    // PDH Power Distribution Implementation
    @Override
    public PDHPort getDriveMotorPDHPort(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        return config != null ? config.drivePowerPort : null;
    }

    @Override
    public PDHPort getSteeringMotorPDHPort(SwerveInstance swerveInstance) {
        SwerveModuleConfig config = SWERVE_CONFIG.get(swerveInstance.label());
        return config != null ? config.steeringPowerPort : null;
    }
    public DeviceInfo vrmUnit1() {
        return new DeviceInfo("VRMUnit1", PowerSource.PDH18);
    }
    
    public DeviceInfo pdhUnit() {
        return new DeviceInfo("PDHUnit", CANBusId.RIO, 1, PowerSource.BATTERY);
    }
    
    public DeviceInfo rioUnit() {
        return new DeviceInfo("RIOUnit", CANBusId.RIO, -1, PowerSource.PDH20);
    }
    
    public DeviceInfo injectorUnit() {
        return new DeviceInfo("InjectorUnit", PowerSource.PDH17);
    }
    
    public DeviceInfo radio() {
        return new DeviceInfo("Radio", PowerSource.INJECTOR);
    }
    public DeviceInfo pneumaticsHub() {
        return new DeviceInfo("PneumaticsHub", CANBusId.RIO, 10, PowerSource.VRM1_12V_2B);
    }
    //Example specification of a solenoid
    //public DeviceInfo getIntakeSolenoid() {
    //    return new DeviceInfo("IntakeSolenoid", 0, PowerSource.PneumaticHub00);
    //}
}
