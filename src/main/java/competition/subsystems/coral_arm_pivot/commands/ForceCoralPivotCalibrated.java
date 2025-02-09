package competition.subsystems.coral_arm_pivot.commands;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import xbot.common.command.BaseCommand;

public class ForceCoralPivotCalibrated extends BaseCommand {

    CoralArmPivotSubsystem coralArmPivotSubsystem;

    public ForceCoralPivotCalibrated(CoralArmPivotSubsystem coralArmPivotSubsystem) {
        this.coralArmPivotSubsystem = coralArmPivotSubsystem;
        this.addRequirements(coralArmPivotSubsystem);
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
