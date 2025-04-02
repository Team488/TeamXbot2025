package competition.subsystems.stinger_arm.commands;

import competition.subsystems.stinger_arm.StingerArmSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

//toggles isCalibrated on and off in coralArmSubsystem for testing
public class ToggleStingerArmCalibratedCommand extends BaseCommand {
    StingerArmSubsystem coralArm;
    boolean calibrationSwitch;

    @Inject
    public ToggleStingerArmCalibratedCommand(StingerArmSubsystem coralArm){
        this.coralArm = coralArm;
        addRequirements(coralArm);
    }
    @Override
    public void initialize() {
        log.info("Initializing..");
        coralArm.setCalibrated(!calibrationSwitch);
        calibrationSwitch = !calibrationSwitch;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
