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
    public boolean isCoralArmPivotMotorReady() {
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

    @Override
    public CANMotorControllerInfo getCoralCollectionMotor() {
        return new CANMotorControllerInfo("CoralCollectionMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                25,
                new CANMotorControllerOutputConfig()
                        .withStatorCurrentLimit(Amps.of(5))
                        .withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.Brake)
        );
    }

    public CANMotorControllerInfo getCoralArmPivotMotor() {
        return new CANMotorControllerInfo("ArmPivotMotor",
                MotorControllerType.TalonFx,
                CANBusId.RIO,
                24,
                new CANMotorControllerOutputConfig().withStatorCurrentLimit(Amps.of(17.5)));
    }

    /*
    @Override
    public CameraInfo[] getCameraInfo() {
        // Robox has no cameras, so return an empty array
        return new CameraInfo[]{};
    }*/
}
