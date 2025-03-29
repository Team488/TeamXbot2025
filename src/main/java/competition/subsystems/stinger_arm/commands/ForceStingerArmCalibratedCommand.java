package competition.subsystems.stinger_arm.commands;

import competition.subsystems.stinger_arm.StingerArmSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ForceStingerArmCalibratedCommand extends BaseCommand {

    StingerArmSubsystem coralArmPivotSubsystem;

    @Inject
    public ForceStingerArmCalibratedCommand(StingerArmSubsystem coralArmPivotSubsystem) {
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
