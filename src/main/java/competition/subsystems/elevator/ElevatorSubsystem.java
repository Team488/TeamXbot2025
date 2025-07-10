package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Distance;

import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.XCANMotorControllerPIDProperties;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.controls.sensors.XLaserCAN;
import xbot.common.properties.PropertyFactory;
import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;

@Singleton
public class ElevatorSubsystem extends BaseSetpointSubsystem<Distance> {

    final ElectricalContract contract;

    public XCANMotorController masterMotor;
    public final XDigitalInput bottomSensor;
    public final XLaserCAN distanceSensor;

    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf,
                             ElectricalContract contract, XDigitalInput.XDigitalInputFactory xDigitalInputFactory,
                             XLaserCAN.XLaserCANFactory xLaserCANFactory) {

        this.contract = contract;
        pf.setPrefix(this);
        
        // based on some initial experiments:
        // Elevator raises 36.375 inches (0.923925 meters) after 42.6535 revolutions
        // 46.16554374 rotations per meter


        if (contract.isElevatorReady()) {
            this.masterMotor = motorFactory.create(
                    contract.getElevatorMotor(), this.getPrefix(), "ElevatorMotorPID",
                    new XCANMotorControllerPIDProperties(
                            4,
                            0,
                            0,
                            0,
                            0.750,
                            1,
                            -0.4)
                    );
            this.registerDataFrameRefreshable(masterMotor);
            masterMotor.setPositionAndVelocityUpdateFrequency(Hertz.of(50));
        }

        if (contract.isElevatorBottomSensorReady()) {
            this.bottomSensor= xDigitalInputFactory.create(contract.getElevatorBottomSensor(), this.getPrefix());
            this.registerDataFrameRefreshable(bottomSensor);
        } else {
            this.bottomSensor = null;
        }

        if (contract.isElevatorDistanceSensorReady()) {
            this.distanceSensor = xLaserCANFactory.create(contract.getElevatorDistanceSensor(), this.getPrefix());
            registerDataFrameRefreshable(distanceSensor);
        } else {
            this.distanceSensor = null;
        }
    }

    @Override
    public void setPower(double power) {
        if (contract.isElevatorReady()) {
            // TODO: set power on the master motor
        }
    }

    @Override
    public Distance getCurrentValue() {
        // TODO: return the current height of the elevator in Meters
        return Meters.zero();
    }

    @Override
    public Distance getTargetValue() {
        // TODO: return the target height of the elevator in Meters
        return Meters.zero();
    }

    @Override
    public void setTargetValue(Distance value) {
        // TODO: remember this target height in this class for later
    }

    @Override
    public boolean isCalibrated() {
        // TODO: return true if the elevator is calibrated, false otherwise
        return false;
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Distance target1, Distance target2) {
        return target1.isEquivalent(target2);
    }


    @Override
    public void periodic() {
        if (contract.isElevatorReady()) {
            masterMotor.periodic();
        }

        aKitLog.record("ElevatorCurrentHeight-m", getCurrentValue().in(Meters));
        aKitLog.record("isElevatorCalibrated", isCalibrated());
        aKitLog.record("isElevatorMaintainerAtGoal", this.isMaintainerAtGoal());
    }
}
