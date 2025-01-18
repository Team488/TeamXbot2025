package competition.subsystems.collection;

import competition.subsystems.AlgaeCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class OutputCommand extends BaseCommand {
    AlgaeCollectionSubsystem algae;

    @Inject
    public OutputCommand(AlgaeCollectionSubsystem algaeCollectionSubsystem) {
        algae = algaeCollectionSubsystem;
        this.addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.output();
    }
}
