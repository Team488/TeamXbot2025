package competition.electrical_contract;

import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.injection.electrical_contract.MotorControllerType;

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
        return true;
    }

    @Override
    public CANMotorControllerInfo getElevatorMotor() {

        CANMotorControllerOutputConfig elevatorMotorConfig = new CANMotorControllerOutputConfig()
                .withStatorCurrentLimit(Amps.of(40))
                .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake)
                .withInversionType(CANMotorControllerOutputConfig.InversionType.Inverted);

        return new CANMotorControllerInfo(
                "ElevatorMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO, 29, //change deviceId later
                elevatorMotorConfig);
    }

    /*
    @Override
    public CameraInfo[] getCameraInfo() {
        // Robox has no cameras, so return an empty array
        return new CameraInfo[]{};
    }*/
}
