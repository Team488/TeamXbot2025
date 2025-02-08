package competition.electrical_contract;

import xbot.common.injection.electrical_contract.CameraInfo;

import javax.inject.Inject;

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
        return true;
    }
/*
    @Override
    public CameraInfo[] getCameraInfo() {
        // Robox has no cameras, so return an empty array
        return new CameraInfo[]{};
    }*/
}
