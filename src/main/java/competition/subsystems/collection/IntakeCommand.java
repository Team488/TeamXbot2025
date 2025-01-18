package competition.subsystems.collection;

import competition.subsystems.AlgaeCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeCommand extends BaseCommand {
    AlgaeCollectionSubsystem algae;

    @Inject
    public IntakeCommand(AlgaeCollectionSubsystem algaeCollectionSubsystem) {
        algae = algaeCollectionSubsystem;
        this.addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.intake();
    }
}
