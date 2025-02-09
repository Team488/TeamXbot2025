package competition.subsystems.coral_arm_pivot.commands;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ForceCoralPivotCalibrated extends BaseCommand {

    CoralArmPivotSubsystem coralArmPivotSubsystem;

    @Inject
    public ForceCoralPivotCalibrated(CoralArmPivotSubsystem coralArmPivotSubsystem) {
        this.coralArmPivotSubsystem = coralArmPivotSubsystem;
    }

    @Override
    public void initialize() {
        coralArmPivotSubsystem.forceCalibrated();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
