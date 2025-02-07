package competition.electrical_contract;

import edu.wpi.first.units.measure.Distance;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.injection.electrical_contract.XCameraElectricalContract;
import xbot.common.injection.electrical_contract.XSwerveDriveElectricalContract;
import xbot.common.injection.swerve.SwerveInstance;
import xbot.common.math.XYPair;

public abstract class ElectricalContract implements XSwerveDriveElectricalContract, XCameraElectricalContract {

    public abstract boolean isDriveReady();

    public abstract boolean areCanCodersReady();

    public abstract CANMotorControllerInfo getDriveMotor(SwerveInstance swerveInstance);

    public abstract CANMotorControllerInfo getSteeringMotor(SwerveInstance swerveInstance);

    public abstract DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance);

    public abstract XYPair getSwerveModuleOffsetsInInches(SwerveInstance swerveInstance);

    public abstract boolean isElevatorReady();

    public abstract CANMotorControllerInfo getElevatorMotor();

    public abstract boolean isElevatorDistanceSensorReady();

    public abstract DeviceInfo getElevatorDistanceSensor();

    public abstract boolean isAlgaeCollectionReady();

    public abstract CANMotorControllerInfo getAlgaeCollectionMotor();

    public abstract boolean isCoralCollectionMotorReady();

    public abstract CANMotorControllerInfo getCoralCollectionMotor();

    public abstract boolean isCoralArmPivotMotorReady();

    public abstract CANMotorControllerInfo getCoralArmPivotMotor();

    public abstract boolean isCoralArmPivotAbsoluteEncoderReady();

    public abstract DeviceInfo getCoralArmPivotAbsoluteEncoder();

    public abstract boolean isCoralArmPivotLowSensorReady();

    public abstract DeviceInfo getCoralArmPivotLowSensor();

    public abstract boolean isCoralSensorReady();

    public abstract DeviceInfo getCoralSensor();

    public abstract boolean isElevatorBottomSensorReady();

    public abstract DeviceInfo getElevatorBottomSensor();

    public abstract boolean isHumanLoadRampReady();

    public abstract CANMotorControllerInfo getAlgaeArmPivotMotor();

    public abstract boolean isAlgaeArmPivotMotorReady();

    public abstract boolean isAlgaeArmBottomSensorReady();

    public abstract DeviceInfo getAlgaeArmBottomSensor();

    public abstract Distance getDistanceFromCenterToOuterBumperX();
}
