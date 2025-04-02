package competition.subsystems.stinger_collection.commands;

import competition.subsystems.stinger_collection.StingerCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ScoreAlgaeCommand extends BaseCommand {
    StingerCollectionSubsystem coral;

    @Inject
    public ScoreAlgaeCommand (StingerCollectionSubsystem coralScorerSubsystem){
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setStingerCollectionState(StingerCollectionSubsystem.StingerCollectionState.SCORING_ALGAE);
    }

    @Override
    public void end(boolean interrupted) {
        coral.setStingerCollectionState(StingerCollectionSubsystem.StingerCollectionState.STOPPED);
    }
}
