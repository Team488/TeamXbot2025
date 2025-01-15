package competition.electrical_contract;

import javax.inject.Inject;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import competition.subsystems.pose.PoseSubsystem;
import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig;
import xbot.common.injection.electrical_contract.CANTalonInfo;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.injection.electrical_contract.MotorControllerType;
import xbot.common.injection.swerve.SwerveInstance;
import xbot.common.math.XYPair;

public class CompetitionContract extends ElectricalContract {
 
    protected final double simulationScalingValue = 256.0 * PoseSubsystem.INCHES_IN_A_METER;

    @Inject
    public CompetitionContract() {}

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
}
