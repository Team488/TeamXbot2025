package competition.subsystems.algae_arm.commands;

import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class RepositionAlgaeArmUp extends BaseCommand {

    AlgaeArmSubsystem algaeArm;

    @Inject
    public RepositionAlgaeArmUp(AlgaeArmSubsystem algaeArm) {
        this.algaeArm = algaeArm;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        algaeArm.goingUp = true;
        algaeArm.repositionToTargetAngle();
    }
}
