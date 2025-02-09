package competition.subsystems.elevator;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

@Singleton
public class ElevatorSubsystem extends BaseSetpointSubsystem<Distance> {

    public enum ElevatorPowerRestrictionReason{
        FullPowerAvailable,
        BottomSensorHit,
        UpperSensorHit,
        Uncalibrated,
        AboveMaxHeight,
        BelowMinHeight
    }

    private double periodicTickCounter;

    final ElectricalContract contract;

    // elevator starts uncalibrated because it could be in the middle of it's range and we have no idea where that is
    private boolean isCalibrated;
    private double elevatorPositionOffset;

    public Distance elevatorTargetHeight;

    public final DoubleProperty rotationsPerMeter;
    public final Distance metersPerRotation;

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

    @Inject
    public ElevatorSubsystem(XCANMotorController.XCANMotorControllerFactory motorFactory, PropertyFactory pf,
                             ElectricalContract contract, XDigitalInput.XDigitalInputFactory xDigitalInputFactory,
                             XLaserCAN.XLaserCANFactory xLaserCANFactory) {

        this.contract = contract;

        this.elevatorPositionOffset = 0.0;

        this.elevatorTargetHeight = Inches.of(0);

        pf.setPrefix(this);

        //these are not real measured heights yet, just placeholders
        l2Height = pf.createPersistentProperty("l2Height", Inches.of(1));
        l3Height = pf.createPersistentProperty("l3Height", Inches.of(15.875));
        l4Height = pf.createPersistentProperty("l4Height", Inches.of(40.651));
        humanLoadHeight = pf.createPersistentProperty("humanLoadHeight", Inches.of(1));
        baseHeight = pf.createPersistentProperty("baseHeight", Inches.of(0));


        //to be tuned
        // based on some initial experiments:
        // Elevator raises 36.375 inches (0.923925 meters) after 42.6535 revolutions
        // 46.16554374 rotations per meter
        double experimentalRotationsPerMeter = 42.6535 / Inches.of(36.375).in(Meters);
        this.rotationsPerMeter = pf.createPersistentProperty("RotationsPerMeter", experimentalRotationsPerMeter);
        this.metersPerRotation = Meters.of(rotationsPerMeter.get() != 0 ? 1.0 / rotationsPerMeter.get() : 0);
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
                        null, null,
                        Seconds.of(8),
                        (state) -> org.littletonrobotics.junction.Logger.recordOutput(this.getPrefix() + "/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setPower(volts.in(Volts) / 12.0),
                        null,
                        this
                )
        );

        if(contract.isElevatorReady()){
            this.masterMotor = motorFactory.create(
                    contract.getElevatorMotor(), this.getPrefix(), "ElevatorMotorPID",
                    new XCANMotorControllerPIDProperties(1,0,0.5)
                    );
            this.registerDataFrameRefreshable(masterMotor);
        }
        if (contract.isElevatorBottomSensorReady()){

            this.bottomSensor= xDigitalInputFactory.create(contract.getElevatorBottomSensor(), "Elevator Bottom Sensor");
            this.registerDataFrameRefreshable(bottomSensor);

        }else{
            this.bottomSensor=null;
        }

        if (contract.isElevatorDistanceSensorReady()) {
            this.distanceSensor = xLaserCANFactory.create(contract.getElevatorDistanceSensor(), this.getPrefix());
            registerDataFrameRefreshable(distanceSensor);
        } else {
            this.distanceSensor = null;
        }

        setCalibrated(false);
    }

    @Override
    public void setPower(double power) {
        if(contract.isElevatorReady()){
            /*if (isTouchingBottom()){
                power = MathUtils.constrainDouble(power,powerWhenBottomSensorHit.get(),1);
            }
            if (belowLowerLimit()){
                power = MathUtils.constrainDouble(power,powerNearLowerLimitThreshold.get(), 1);
            }
            if (aboveUpperLimit()){
                power = MathUtils.constrainDouble(power, -1, powerNearUpperLimitThreshold.get());
            }
            if (!isCalibrated){
                power = MathUtils.constrainDouble(power,calibrationNegativePower.get(),0);
            }*/

            aKitLog.record("ElevatorPower", power);

            masterMotor.setVoltage(Volts.of(power*12));
        }
    }

    public void markElevatorAsCalibratedAgainstLowerLimit(){
        isCalibrated = true;
        elevatorPositionOffset = this.masterMotor.getPosition().in(Rotations);
    }

    @Override
    public Distance getCurrentValue() {
        Distance currentHeight = Meters.of(0);
        if (contract.isElevatorReady()){
            currentHeight = Meters.of(
                    (this.masterMotor.getPosition().in(Rotations) - elevatorPositionOffset) * metersPerRotation.in(Meters));
        }
        return currentHeight;
    }

    public LinearVelocity getCurrentVelocity() {
        return MetersPerSecond.of(masterMotor.getVelocity().in(RotationsPerSecond) * metersPerRotation.in(Meters));
    }

    @Override
    public Distance getTargetValue() {
       return elevatorTargetHeight;
    }

    @Override
    public void setTargetValue(Distance value) {
       elevatorTargetHeight = value;
    }

    public void setTargetHeight(Landmarks.CoralLevel value){
        switch (value){
            case TWO -> setTargetValue(l2Height.get());
            case THREE -> setTargetValue(l3Height.get());
            case FOUR -> setTargetValue(l4Height.get());
            case COLLECTING -> setTargetValue(humanLoadHeight.get());
            default -> setTargetValue(baseHeight.get());
        }
    }

    public boolean isTouchingBottom(){
        if (contract.isElevatorBottomSensorReady()){
            return this.bottomSensor.get();
        }
        return false;
    }

    public boolean aboveUpperLimit(){
        return getCurrentValue().in(Meters) > upperHeightLimit.get().in(Meters);
    }

    public boolean belowLowerLimit(){
        return getCurrentValue().in(Meters) < lowerHeightLimit.get().in(Meters);
    }

    public void setCalibrated(boolean calibrated){
        isCalibrated = calibrated;
    }

    @Override
    public boolean isCalibrated() {
        return isCalibrated;
    }

    private Distance getRawDistance() {
        if (contract.isElevatorDistanceSensorReady()) {
            return distanceSensor.getDistance();
        } else {
            return Meter.of(0);
        }
    }

    @Override
    protected boolean areTwoTargetsEquivalent(Distance target1, Distance target2) {
        return target1.isEquivalent(target2);
    }

    /**
     * Gets a command to run the SysId routine in the quasistatic mode.
     * @param direction The direction to run the SysId routine.
     * @return The command to run the SysId routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /**
     * Gets a command to run the SysId routine in the dynamic mode.
     * @param direction The direction to run the SysId routine.
     * @return The command to run the SysId routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public void periodic(){
        if (contract.isElevatorReady()){
            masterMotor.periodic();
        }
        //bandage case: isTouchingBottom flashes true for one tick on startup, investigate later?
        if (this.isTouchingBottom() && periodicTickCounter >= 3){
            markElevatorAsCalibratedAgainstLowerLimit();
        }
        aKitLog.record("ElevatorTargetHeight-m",elevatorTargetHeight);
        aKitLog.record("ElevatorCurrentHeight-m",getCurrentValue().in(Meters));
        aKitLog.record("ElevatorBottomSensor",this.isTouchingBottom());
        aKitLog.record("isElevatorCalibrated", isCalibrated());
        aKitLog.record("ElevatorDistanceSensor-m",getRawDistance().in(Meters));

        periodicTickCounter++;
    }


}
