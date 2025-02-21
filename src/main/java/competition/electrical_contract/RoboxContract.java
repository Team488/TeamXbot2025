package competition.electrical_contract;

import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.CameraInfo;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Amps;

public class RoboxContract extends Contract2025 {
    @Inject
    public RoboxContract() {}

    @Override
    public boolean isDriveReady() {
        return false;
    }

    @Override
    public boolean areCanCodersReady() {
        return false;
    }

    @Override
    public boolean isElevatorReady() {
        return false;
    }

    @Override
    public boolean isCoralCollectionMotorReady() {
        return false;
    }

    @Override
    public boolean isElevatorBottomSensorReady() {
        return false;
    }

    @Override
    public boolean isCoralArmMotorReady() {
        return false;
    }

    @Override
    public CameraInfo[] getCameraInfo() {
        return new CameraInfo[] {};
    }

    public boolean isAlgaeCollectionReady() { return false; }
}
