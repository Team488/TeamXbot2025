package competition.subsystems.algae_collection.commands;

import competition.subsystems.algae_collection.AlgaeCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class AlgaeCollectionIntakeCommand extends BaseCommand {
    AlgaeCollectionSubsystem algae;

    @Inject
    public AlgaeCollectionIntakeCommand(AlgaeCollectionSubsystem algaeCollectionSubsystem) {
        algae = algaeCollectionSubsystem;
        this.addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.intake();
    }
}
