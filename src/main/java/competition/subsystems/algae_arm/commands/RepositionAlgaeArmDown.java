package competition.subsystems.algae_arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;


public class RepositionAlgaeArmDown extends BaseCommand {
    AlgaeArmSubsystem algaeArm;

    @Inject
    public RepositionAlgaeArmDown(AlgaeArmSubsystem algaeArm, OperatorInterface oi) {
        this.algaeArm = algaeArm;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        algaeArm.goingUp = false;
        algaeArm.repositionToTargetAngle();
    }
}
