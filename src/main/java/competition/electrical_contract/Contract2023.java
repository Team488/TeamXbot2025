package competition.electrical_contract;

import competition.subsystems.pose.PoseSubsystem;
import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.injection.electrical_contract.MotorControllerType;
import xbot.common.injection.swerve.SwerveInstance;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import java.util.Map;

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
        "FrontLeftDrive",  new SwerveModuleConfig(31, 30, 51, new XYPair(15, 15)),
        "FrontRightDrive", new SwerveModuleConfig(29, 28, 52, new XYPair(15, -15)),
        "RearLeftDrive",   new SwerveModuleConfig(38, 39, 53, new XYPair(-15, 15)),
        "RearRightDrive",  new SwerveModuleConfig(21, 20, 54, new XYPair(-15, -15))
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
    public CameraInfo[] getCameraInfo() {
        // 2023 has no cameras, so return an empty array
        return new CameraInfo[]{};
    }
}
