package competition.subsystems.algae_arm.commands;

import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ForceAlgaeArmCalibrated extends BaseCommand {

    AlgaeArmSubsystem algaeArmSubsystem;

    @Inject
    public ForceAlgaeArmCalibrated(AlgaeArmSubsystem algaeArmSubsystem) {
        this.algaeArmSubsystem = algaeArmSubsystem;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        algaeArmSubsystem.forceCalibratedHere();
        algaeArmSubsystem.setTargetValue(algaeArmSubsystem.getCurrentValue());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}