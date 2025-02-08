package competition.subsystems.coral_arm_pivot.commands;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

//toggles isCalibrated on and off in coralArmSubsystem for testing
public class ToggleCoralArmCalibratedCommand extends BaseCommand {
    CoralArmPivotSubsystem coralArm;
    boolean calibrationSwitch;

    @Inject
    public ToggleCoralArmCalibratedCommand(CoralArmPivotSubsystem coralArm){
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
