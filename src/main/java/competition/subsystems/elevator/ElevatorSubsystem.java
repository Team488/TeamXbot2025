package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import competition.motion.ComplimentaryFilter;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
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
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DistanceProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.Property.PropertyLevel;

import javax.inject.Inject;
import javax.inject.Singleton;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
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
    private final MutDistance laserCANPositionOffset = Meters.mutable(0);
    private final MutAngle elevatorMotorPositionOffset = Degrees.mutable(0);

    public final MutDistance elevatorTargetHeight = Inches.mutable(0);

    public final DoubleProperty rotationsPerMeter;

    public final DoubleProperty calibrationNegativePower;
    public final DoubleProperty powerNearLowerLimitThreshold;
    public final DoubleProperty powerNearUpperLimitThreshold;
    public final DoubleProperty powerWhenBottomSensorHit;

    public final DoubleProperty motionMagicAcceleration;
    public final DoubleProperty motionMagicJerk;

    public final BooleanProperty motionMagicEnabled;

    public XCANMotorController masterMotor;

    public final DistanceProperty upperHeightLimit;
    public final DistanceProperty lowerHeightLimit;

    //important heights
    public final DistanceProperty l2Height;
    public final DistanceProperty l3Height;
    public final DistanceProperty l4Height;
    public final DistanceProperty humanLoadHeight;
    public final DistanceProperty baseHeight;
    public final DistanceProperty trimValue;
    public final DistanceProperty trimChangeAmount;

    public final DoubleProperty laserCanMaxMeasurementLatency;

    public final XDigitalInput bottomSensor;
    public final XLaserCAN distanceSensor;

    private final SysIdRoutine sysId;

    private final ComplimentaryFilter sensorFusionFilter;

    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf,
                             ElectricalContract contract, XDigitalInput.XDigitalInputFactory xDigitalInputFactory,
                             XLaserCAN.XLaserCANFactory xLaserCANFactory) {

        this.contract = contract;

        sensorFusionFilter = new ComplimentaryFilter(pf, this.getPrefix(), true, 0.5);

        pf.setPrefix(this);

        l2Height = pf.createPersistentProperty("l2Height", Inches.of(1.5));
        l3Height = pf.createPersistentProperty("l3Height", Inches.of(16.25));
        l4Height = pf.createPersistentProperty("l4Height", Inches.of(47.5));
        humanLoadHeight = pf.createPersistentProperty("humanLoadHeight", Inches.of(1));
        pf.setDefaultLevel(PropertyLevel.Debug);
        baseHeight = pf.createPersistentProperty("baseHeight", Inches.of(0));
        trimValue = pf.createPersistentProperty("trimValue",Inches.of(0));
        trimChangeAmount = pf.createPersistentProperty("TrimUpAmount", Inches.of(1));
        laserCanMaxMeasurementLatency = pf.createPersistentProperty("laserCanMaxMeasurementLatency-S", 0.04);


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

        this.motionMagicAcceleration = pf.createPersistentProperty("motionMagicMaxAcceleration", 1);
        this.motionMagicJerk = pf.createPersistentProperty("motionMagicMaxJerk", 0.1);

        this.motionMagicEnabled = pf.createPersistentProperty("motionMagicEnabled", false);
        pf.setDefaultLevel(PropertyLevel.Important);


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
            configureMotionMagicConstraints();
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

    /**
     * This method sets the LaserCAN sensor offset if the sensor is reporting a
     * valid value. This should only be called when the elevator is touching the
     * bottom sensor.
     * @return true if the offset was set successfully
     */
    private boolean trySetLaserCANOffset() {
        var distance = getRawLaserDistance();
        distance.ifPresent(laserCANPositionOffset::mut_replace);
        return distance.isPresent();
    }

    private boolean trySetMotorOffset() {
        var motorRotations = getRawMotorAngle();
        if (motorRotations != null) {
            elevatorMotorPositionOffset.mut_replace(motorRotations);
            return true;
        }
        return false;
    }

    /**
     * This method attempts to calibrate the elevator against the lower limit. It
     * will set the laserCAN offset and the motor offset if the sensor is reporting a
     * valid value. This should only be called when the elevator is touching the
     * bottom sensor.
     * @return true if the calibration was successful
     */
    public boolean tryMarkElevatorCalibratedAgainstLowerLimit() {
        var success = trySetLaserCANOffset() && trySetMotorOffset();
        if (success) {
            isCalibrated = true;
        }
        return success;
    }

    private boolean tryCalibrateMotorOffsetViaLaserCAN() {
        if (!isCalibrated) {
            return false;
        }
        var laserDistance = getCalibratedLaserDistance();
        laserDistance.ifPresent(d -> {
            var motorRotations = getRawMotorAngle();
            var motorRotationsToZero = Rotations.of(d.in(Meters) / getMetersPerRotation().in(Meters));
            elevatorMotorPositionOffset.mut_replace(motorRotations.minus(motorRotationsToZero));
        });
        return laserDistance.isPresent();
    }

    @Override
    public Distance getCurrentValue() {
        return Meters.of(sensorFusionFilter.calculateFilteredValue(
                getCalibratedLaserDistance().map(d -> d.in(Meters)).orElse(Double.MAX_VALUE),
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
        elevatorTargetHeight.mut_replace(value);
    }

    public void setTargetHeight(Landmarks.CoralLevel value) {
        switch (value) {
            case TWO -> setTargetValue(l2Height.get());
            case THREE -> setTargetValue(l3Height.get().plus(trimValue.get()));
            case FOUR -> setTargetValue(l4Height.get().plus(trimValue.get()));
            case COLLECTING_1_CORAL_BACK -> setTargetValue(humanLoadHeight.get());
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

    private Optional<Distance> getCalibratedLaserDistance() {
        return getRawLaserDistance().map(d -> d.minus(laserCANPositionOffset));
    }

    private Optional<Distance> getRawLaserDistance() {
        if (contract.isElevatorDistanceSensorReady()) {
            var distance = distanceSensor.getDistance();
            var latency = distanceSensor.getMeasurementLatency();
            return distance.filter(d -> latency.lt(Seconds.of(laserCanMaxMeasurementLatency.get())));
        }
        return Optional.empty();
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
        setElevatorDeltaFromCurrentHeight(Meters.of(heightInMeters).minus(getCurrentValue()));
    }

    public void setElevatorDeltaFromCurrentHeight(Distance heightDelta) {
        // Much like swerve steering, the maintainer will give us the error (e.g.
        // "you need to move up 0.25 meters"). The elevator will need to first convert
        // that to rotations, and then add it to its current number of rotations, and
        // THAT value is set as a target for onboard PID.
        var deltaRotations = Rotations.of(heightDelta.in(Meters) * rotationsPerMeter.get());
        masterMotor.setPositionTarget(
                masterMotor.getPosition().plus(deltaRotations),
                motionMagicEnabled.get()
                ? XCANMotorController.MotorPidMode.TrapezoidalVoltage
                : XCANMotorController.MotorPidMode.Voltage);
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

    public void trimElevatorUp(){
        trimValue.set(trimValue.get().plus(Inches.of(trimChangeAmount.get().in(Inches))));
    }

    public void trimElevatorDown(){
        trimValue.set(trimValue.get().minus((Inches.of(trimChangeAmount.get().in(Inches)))));
    }

    public void toggleMotionMagic(){
        motionMagicEnabled.set(!motionMagicEnabled.get());
    }

    public boolean isMotionMagicEnabled(){
        return motionMagicEnabled.get();
    }

    public void configureMotionMagicConstraints(){
        masterMotor.setTrapezoidalProfileAcceleration(RadiansPerSecondPerSecond.of(motionMagicAcceleration.get()));
        masterMotor.setTrapezoidalProfileJerk(RadiansPerSecondPerSecond.of(motionMagicJerk.get()).per(Second));
    }

    @Override
    public void periodic() {
        if (contract.isElevatorReady()) {
            masterMotor.periodic();
        }
        if(motionMagicAcceleration.hasChangedSinceLastCheck() || motionMagicJerk.hasChangedSinceLastCheck()){
            configureMotionMagicConstraints();
        }
        //bandage case: isTouchingBottom flashes true for one tick on startup, investigate later?
        if (this.isTouchingBottom() && periodicTickCounter >= 3 && !isCalibrated()) {
            // Calibration may fail if the LaserCAN is not reporting a valid value
            if (tryMarkElevatorCalibratedAgainstLowerLimit()) {
                setTargetValue(getCurrentValue());
            }
        }

        aKitLog.record("ElevatorTrimValue", trimValue.get().in(Meters));
        aKitLog.record("ElevatorTargetHeight-m", elevatorTargetHeight.in(Meters));
        aKitLog.record("ElevatorCurrentHeight-m", getCurrentValue().in(Meters));
        aKitLog.record("ElevatorBottomSensor", this.isTouchingBottom());
        aKitLog.record("isElevatorCalibrated", isCalibrated());
        aKitLog.record("isElevatorMaintainerAtGoal", this.isMaintainerAtGoal());
        isNotCalibratedAlert.set(!isCalibrated());
        getRawLaserDistance().ifPresent(d -> aKitLog.record("ElevatorDistanceSensor-m", d.in(Meters)));
        getCalibratedLaserDistance().ifPresent(d -> aKitLog.record("CalibratedElevatorDistanceSensor-m", d.in(Meters)));
        aKitLog.record("CalibratedElevatorMotorSensor-m", getCalibratedMotorDistance().in(Meters));
        aKitLog.record("MotorOffset-rotations", elevatorMotorPositionOffset.in(Rotations));
        aKitLog.record("LaserCANOffset", laserCANPositionOffset.in(Meters));

        periodicTickCounter++;
    }
}
