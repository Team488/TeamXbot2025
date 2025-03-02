package competition.subsystems.climber;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.units.measure.Time;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.XSolenoid;
import xbot.common.controls.sensors.XTimer;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

@Singleton
public class ClimberSubsystem extends BaseSubsystem {

    public enum PawlState{
        RETRACTED,
        EXTENDING,
        ONCOOLDOWN,
        EXTENDED,
    }

    public Time pawlTimestampStart = XTimer.getFPGATimestampTime();
    public Time pawlTimestampEnd = pawlTimestampStart.plus(Seconds.of(1.5));
    public Time pawlTimestampCooldown = pawlTimestampEnd.plus(Seconds.of(6));

    public final XCANMotorController climberMotor;
    public final DoubleProperty climberPower;
    public final ElectricalContract contract;

    public final XCANMotorController pawlMotor;

    public PawlState currentPawlState;

    @Inject
    public ClimberSubsystem(XCANMotorController.XCANMotorControllerFactory xcanMotorControllerFactory,
                            ElectricalContract contract, PropertyFactory pf) {
        pf.setPrefix(this);
        if (contract.isClimberMotorReady()) {
            this.climberMotor = xcanMotorControllerFactory.create(contract.getClimberMotor(),
                    getPrefix(), "ClimberMotor");
            this.registerDataFrameRefreshable(climberMotor);
        } else {
            this.climberMotor = null;
        }

        if (contract.isPawlMotorReady()){
            this.pawlMotor = xcanMotorControllerFactory.create(contract.getPawlMotor(),
                    getPrefix(), "PawlMotor");
            this.registerDataFrameRefreshable(pawlMotor);
        } else{
            this.pawlMotor = null;
        }

        this.contract = contract;

        this.climberPower = pf.createPersistentProperty("climberPower", 0.1);
    }

    public void setPower(double power) {
        if (contract.isClimberMotorReady()) {
            this.climberMotor.setPower(power);
        }
    }

    public void setPawlMotorPower(double power){
        if(contract.isPawlMotorReady()){
            this.pawlMotor.setVoltage(Volts.of(power * 12));
        }
    }

    public void stop() {
        if (contract.isClimberMotorReady()) {
            this.climberMotor.setPower(0);
        }
    }

    public void releaseClimberSolenoid(){
        currentPawlState = decideState();

        switch(currentPawlState){
            case EXTENDING -> setPawlMotorPower(1);
            case ONCOOLDOWN -> setPawlMotorPower(0);
        }
    }

    public void resetPawlTimestampStart(){
        pawlTimestampStart = XTimer.getFPGATimestampTime();
    }

    public PawlState decideState(){
        pawlTimestampEnd = pawlTimestampStart.plus(Seconds.of(1.5));
        pawlTimestampCooldown = pawlTimestampEnd.plus(Seconds.of(6));
        
        if (XTimer.getFPGATimestampTime().lt(pawlTimestampEnd)){
            return PawlState.EXTENDING;
        }

        if (XTimer.getFPGATimestampTime().lt(pawlTimestampCooldown) && XTimer.getFPGATimestampTime().gt(pawlTimestampEnd)){
            return PawlState.ONCOOLDOWN;
        }

        if (XTimer.getFPGATimestampTime().lt(pawlTimestampCooldown)){
            return PawlState.EXTENDED;
        }

        return PawlState.RETRACTED;
    }

    @Override
    public void periodic() {
        if (currentPawlState == PawlState.ONCOOLDOWN){
            setPawlMotorPower(0);
        }
    }
}
