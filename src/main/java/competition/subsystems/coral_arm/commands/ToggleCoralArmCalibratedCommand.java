package competition.subsystems.coral_arm.commands;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

//toggles isCalibrated on and off in coralArmSubsystem for testing
public class ToggleCoralArmCalibratedCommand extends BaseCommand {
    CoralArmSubsystem coralArm;
    boolean calibrationSwitch;

    @Inject
    public ToggleCoralArmCalibratedCommand(CoralArmSubsystem coralArm){
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
