package competition.subsystems.algae_collection.commands;

import competition.subsystems.algae_collection.AlgaeCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class AlgaeCollectionOutputCommand extends BaseCommand {
    AlgaeCollectionSubsystem algae;

    @Inject
    public AlgaeCollectionOutputCommand(AlgaeCollectionSubsystem algaeCollectionSubsystem) {
        algae = algaeCollectionSubsystem;
        this.addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.output();
    }
}
