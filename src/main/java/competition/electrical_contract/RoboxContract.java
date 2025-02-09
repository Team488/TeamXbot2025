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
    public boolean isCoralCollectionMotorReady() {
        return true;
    }

    @Override
    public boolean isElevatorBottomSensorReady() {
        return false;
    }

    @Override
    public boolean isCoralArmMotorReady() {
        return true;
    }

    @Override
    public CANMotorControllerInfo getElevatorMotor() {

        var motor2025 = super.getElevatorMotor();

        return new CANMotorControllerInfo(
                motor2025.name(),
                motor2025.type(),
                CANBusId.RIO, // Only relevant when connected directly to the robox.
                29, // TODO: This is not the correct long-term value. The motor needs to be given a new ID.
                motor2025.outputConfig().withStatorCurrentLimit(Amps.of(40)));
                // When running off of the wall power supply, the current limit is 40A.
    }

    @Override
    public CANMotorControllerInfo getCoralCollectionMotor() {

        var motor2025 = super.getCoralCollectionMotor();

        return new CANMotorControllerInfo(
                motor2025.name(),
                motor2025.type(),
                CANBusId.RIO, // Only relevant when connected directly to the robox.
                motor2025.deviceId(), // TODO: This is not the correct long-term value. The motor needs to be given a new ID.
                motor2025.outputConfig().withStatorCurrentLimit(Amps.of(5)) // limiting current on the robox
        );
    }

    public CANMotorControllerInfo getCoralArmPivotMotor() {

        var motor2025 = super.getCoralArmPivotMotor();

        return new CANMotorControllerInfo(
                motor2025.name(),
                motor2025.type(),
                CANBusId.RIO, // Only relevant when connected directly to the robox.
                motor2025.deviceId(), // TODO: This is not the correct long-term value. The motor needs to be given a new ID.
                motor2025.outputConfig().withStatorCurrentLimit(Amps.of(17.5)) // limiting current on the robox
        );
    }

    @Override
    public CameraInfo[] getCameraInfo() {
        return new CameraInfo[] {};
    }
}
