package competition.electrical_contract;

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

    public abstract boolean isAlgaeCollectionReady();

    public abstract CANMotorControllerInfo getAlgaeCollectionMotor();

    public abstract boolean isCoralCollectionMotorReady();

    public abstract CANMotorControllerInfo getCoralCollectionMotor();
}
