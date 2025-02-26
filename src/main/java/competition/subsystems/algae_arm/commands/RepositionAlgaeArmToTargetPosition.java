package competition.subsystems.algae_arm.commands;

import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import org.checkerframework.checker.i18nformatter.qual.I18nChecksFormat;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class RepositionAlgaeArmToTargetPosition extends BaseCommand {

    AlgaeArmSubsystem algaeArm;

    @Inject
    public RepositionAlgaeArmToTargetPosition(AlgaeArmSubsystem algaeArm) {
        this.algaeArm = algaeArm;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }
}
