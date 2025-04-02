package competition.subsystems.stinger_collection.commands;

import competition.subsystems.stinger_collection.StingerCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class StopCoralCommand extends BaseCommand {
    StingerCollectionSubsystem coral;

    @Inject
    public StopCoralCommand(StingerCollectionSubsystem coralScorerSubsystem){
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setStingerCollectionState(StingerCollectionSubsystem.StingerCollectionState.STOPPED);
    }
}
