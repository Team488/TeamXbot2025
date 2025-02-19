package competition.subsystems.coral_arm.commands;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ForceCoralPivotCalibrated extends BaseCommand {

    CoralArmSubsystem coralArmPivotSubsystem;

    @Inject
    public ForceCoralPivotCalibrated(CoralArmSubsystem coralArmPivotSubsystem) {
        this.coralArmPivotSubsystem = coralArmPivotSubsystem;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        coralArmPivotSubsystem.forceCalibratedHere();
        coralArmPivotSubsystem.setTargetValue(coralArmPivotSubsystem.getCurrentValue());
        this.setRunsWhenDisabled(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
