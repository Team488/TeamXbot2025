package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import competition.motion.ComplimentaryFilter;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.XCANMotorControllerPIDProperties;
import xbot.common.controls.sensors.XDigitalInput;
import xbot.common.controls.sensors.XLaserCAN;
import xbot.common.math.MathUtils;
import xbot.common.properties.DistanceProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

@Singleton
public class ElevatorSubsystem extends BaseSetpointSubsystem<Distance> {

    private double periodicTickCounter;

    final ElectricalContract contract;

    // elevator starts uncalibrated because it could be in the middle of it's range and we have no idea where that is
    private boolean isCalibrated;
    final Alert isNotCalibratedAlert = new Alert("Elevator: not calibrated", Alert.AlertType.kWarning);
    private Distance laserCANPositionOffset;
    private Angle elevatorMotorPositionOffset;

    public Distance elevatorTargetHeight;

    public final DoubleProperty rotationsPerMeter;

    public final DoubleProperty calibrationNegativePower;
    public final DoubleProperty powerNearLowerLimitThreshold;
    public final DoubleProperty powerNearUpperLimitThreshold;
    public final DoubleProperty powerWhenBottomSensorHit;


    public XCANMotorController masterMotor;

    public final DistanceProperty upperHeightLimit;
    public final DistanceProperty lowerHeightLimit;

    //important heights
    public final DistanceProperty l2Height;
    public final DistanceProperty l3Height;
    public final DistanceProperty l4Height;
    public final DistanceProperty humanLoadHeight;
    public final DistanceProperty baseHeight;

    public final XDigitalInput bottomSensor;
    public final XLaserCAN distanceSensor;

    private final SysIdRoutine sysId;

    private final ComplimentaryFilter sensorFusionFilter;

    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf,
                             ElectricalContract contract, XDigitalInput.XDigitalInputFactory xDigitalInputFactory,
                             XLaserCAN.XLaserCANFactory xLaserCANFactory) {

        this.contract = contract;

        this.laserCANPositionOffset = Meters.zero();
        this.elevatorMotorPositionOffset = Rotations.zero();
        this.elevatorTargetHeight = Inches.of(0);
        sensorFusionFilter = new ComplimentaryFilter(pf, this.getPrefix(), true, 0.5);

        pf.setPrefix(this);

        l2Height = pf.createPersistentProperty("l2Height", Inches.of(7.0));
        l3Height = pf.createPersistentProperty("l3Height", Inches.of(23.0));
        l4Height = pf.createPersistentProperty("l4Height", Inches.of(46.0));
        humanLoadHeight = pf.createPersistentProperty("humanLoadHeight", Inches.of(1));
        baseHeight = pf.createPersistentProperty("baseHeight", Inches.of(0));


        //to be tuned
        // based on some initial experiments:
        // Elevator raises 36.375 inches (0.923925 meters) after 42.6535 revolutions
        // 46.16554374 rotations per meter
        double experimentalRotationsPerMeter = 42.6535 / Inches.of(36.375).in(Meters);
        this.rotationsPerMeter = pf.createPersistentProperty("RotationsPerMeter", experimentalRotationsPerMeter);
        if (rotationsPerMeter.get() == 0){log.warn("ROTATIONS PER METER CANNOT BE ZERO CHANGE THIS NOW PLEASE");}

        this.calibrationNegativePower = pf.createPersistentProperty("calibrationNegativePower", -0.05);

        //power limits near max and min height
        this.upperHeightLimit = pf.createPersistentProperty("upperHeightLimit", l4Height.get());
        this.lowerHeightLimit = pf.createPersistentProperty("lowerHeightLimit", baseHeight.get());
        this.powerNearUpperLimitThreshold = pf.createPersistentProperty("powerNearUpperLimit", 0.0);
        this.powerNearLowerLimitThreshold = pf.createPersistentProperty("powerNearLowerLimit", 0.0);
        this.powerWhenBottomSensorHit = pf.createPersistentProperty("powerWhenBottomSensorHit", 0);

        this.sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.2).per(Second),
                        Volts.of(0.5),
                        Seconds.of(8),
                        (state) -> org.littletonrobotics.junction.Logger.recordOutput(this.getPrefix() + "/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setPower(volts.in(Volts) / 12.0),
                        null,
                        this
                )
        );

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

        if (contract.isElevatorReady() && contract.isElevatorBottomSensorReady()) {
            this.masterMotor.setSoftwareReverseLimit(this::isTouchingBottom);
        }
    }

    @Override
    public void setPower(double power) {
        if (contract.isElevatorReady()) {
            if (isTouchingBottom()) {
                power = MathUtils.constrainDouble(power, powerWhenBottomSensorHit.get(), 1);
            }
            if (belowLowerLimit()) {
                power = MathUtils.constrainDouble(power, powerNearLowerLimitThreshold.get(), 1);
            }
            if (aboveUpperLimit()) {
                power = MathUtils.constrainDouble(power, -1, powerNearUpperLimitThreshold.get());
            }
            if (!isCalibrated) {
                power = MathUtils.constrainDouble(power, calibrationNegativePower.get(), 0);
            }

            masterMotor.setVoltage(Volts.of(power*12));
        }
    }

    public void markElevatorAsCalibratedAgainstLowerLimit() {
        isCalibrated = true;
        if (this.masterMotor != null) {
            laserCANPositionOffset = getRawLaserDistance();
            elevatorMotorPositionOffset = getRawMotorAngle();
        } else {
            laserCANPositionOffset = Meters.zero();
            elevatorMotorPositionOffset = Rotations.zero();
        }
    }

    private void calibrateElevatorMotorOffsetViaLaserCAN() {
        var laserDistance = getCalibratedLaserDistance();
        var motorRotations = getRawMotorAngle();
        var motorRotationsToZero = Rotations.of(laserDistance.in(Meters) / getMetersPerRotation().in(Meters));
        elevatorMotorPositionOffset = motorRotations.minus(motorRotationsToZero);
    }

    @Override
    public Distance getCurrentValue() {
        return Meters.of(sensorFusionFilter.calculateFilteredValue(
                getCalibratedLaserDistance().in(Meters),
                getCalibratedMotorDistance().in(Meters)));
    }

    public LinearVelocity getCurrentVelocity() {
        if (masterMotor != null) {
            return MetersPerSecond.of(masterMotor.getVelocity().in(RotationsPerSecond) * getMetersPerRotation().in(Meters));
        } else {
            return MetersPerSecond.of(0);
        }
    }

    @Override
    public Distance getTargetValue() {
        return elevatorTargetHeight;
    }

    @Override
    public void setTargetValue(Distance value) {
        elevatorTargetHeight = value;
    }

    public void setTargetHeight(Landmarks.CoralLevel value) {
        switch (value) {
            case TWO -> setTargetValue(l2Height.get());
            case THREE -> setTargetValue(l3Height.get());
            case FOUR -> setTargetValue(l4Height.get());
            case COLLECTING -> setTargetValue(humanLoadHeight.get());
            default -> setTargetValue(baseHeight.get());
        }
    }

    public boolean isTouchingBottom() {
        if (contract.isElevatorBottomSensorReady()) {
            return this.bottomSensor.get();
        }
        return false;
    }

    public boolean aboveUpperLimit() {
        return getCurrentValue().in(Meters) > upperHeightLimit.get().in(Meters);
    }

    public boolean belowLowerLimit() {
        return getCurrentValue().in(Meters) < lowerHeightLimit.get().in(Meters);
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    private Distance getCalibratedLaserDistance() {
        return getRawLaserDistance().minus(laserCANPositionOffset);
    }

    private Distance getRawLaserDistance() {
        if (contract.isElevatorDistanceSensorReady()) {
            var distance = distanceSensor.getDistance();
            if (distance != null) {
                return distance;
            }
        }
        return Meter.of(0);
    }

    private Angle getRawMotorAngle() {
        if (contract.isElevatorReady()) {
            return masterMotor.getPosition();
        }
        return Rotations.zero();
    }

    private Angle getCalibratedMotorAngle() {
        return getRawMotorAngle().minus(elevatorMotorPositionOffset);
    }

    private Distance getCalibratedMotorDistance() {
        return Meters.of(getCalibratedMotorAngle().in(Rotations) * getMetersPerRotation().in(Meters));
    }

    private Distance getMetersPerRotation() {
        return Meters.of(rotationsPerMeter.get() != 0 ? 1.0 / rotationsPerMeter.get() : 0);
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Distance target1, Distance target2) {
        return target1.isEquivalent(target2);
    }

    public void setElevatorHeightGoalOnMotor(double heightInMeters) {
        // If we are in the LaserCAN range, we use that information directly via a delta approach.

        var targetRotations = Rotations.of(heightInMeters * rotationsPerMeter.get()
                + elevatorMotorPositionOffset.in(Rotations));
        masterMotor.setPositionTarget(targetRotations, XCANMotorController.MotorPidMode.Voltage);

        /*
        var currentHeight = getCurrentValue();
        if (currentHeight.lt(Meters.of(laserCANMaxTrustedHeight.get()))) {
            // We are in the lower part of the elevator where we can use the LaserCAN
            // directly.
            // compute the height delta:
            var error = Meters.of(heightInMeters).minus(currentHeight);
            // servo to that delta
            setElevatorDeltaFromCurrentHeight(error);
        } else {
            // We are way up on the elevator. We need to use the onboard motor rotations instead.

        }*/
    }

    public void setElevatorDeltaFromCurrentHeight(Distance heightDelta) {
        // Much like swerve steering, the maintainer will give us the error (e.g.
        // "you need to move up 0.25 meters"). The elevator will need to first convert
        // that to rotations, and then add it to its current number of rotations, and
        // THAT value is set as a target for onboard PID.

        var deltaRotations = Rotations.of(heightDelta.in(Meters) * rotationsPerMeter.get());
        masterMotor.setPositionTarget(
                masterMotor.getPosition().plus(deltaRotations),
                XCANMotorController.MotorPidMode.Voltage);
    }

    /**
     * Gets a command to run the SysId routine in the quasistatic mode.
     *
     * @param direction The direction to run the SysId routine.
     * @return The command to run the SysId routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /**
     * Gets a command to run the SysId routine in the dynamic mode.
     *
     * @param direction The direction to run the SysId routine.
     * @return The command to run the SysId routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (contract.isElevatorReady()) {
            masterMotor.periodic();
        }
        //bandage case: isTouchingBottom flashes true for one tick on startup, investigate later?
        if (this.isTouchingBottom() && periodicTickCounter >= 3 && !isCalibrated()) {
            markElevatorAsCalibratedAgainstLowerLimit();
            setTargetValue(getCurrentValue());
        }

        aKitLog.record("ElevatorTargetHeight-m", elevatorTargetHeight);
        aKitLog.record("ElevatorCurrentHeight-m", getCurrentValue().in(Meters));
        aKitLog.record("ElevatorBottomSensor", this.isTouchingBottom());
        aKitLog.record("isElevatorCalibrated", isCalibrated());
        aKitLog.record("isElevatorMaintainerAtGoal", this.isMaintainerAtGoal());
        isNotCalibratedAlert.set(!isCalibrated());
        aKitLog.record("ElevatorDistanceSensor-m", getRawLaserDistance().in(Meters));
        aKitLog.record("CalibratedElevatorDistanceSensor-m", getCalibratedLaserDistance().in(Meters));
        aKitLog.record("CalibratedElevatorMotorSensor-m", getCalibratedMotorDistance().in(Meters));
        aKitLog.record("MotorOffset-rotations", elevatorMotorPositionOffset.in(Rotations));

        periodicTickCounter++;
    }


}
