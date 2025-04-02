package competition.subsystems.stinger_collection.commands;

import competition.subsystems.stinger_collection.StingerCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeUntilCoralCollectedCommand extends BaseCommand {
    StingerCollectionSubsystem coral;

    @Inject
    public IntakeUntilCoralCollectedCommand(StingerCollectionSubsystem coralScorerSubsystem){
        coral= coralScorerSubsystem;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setStingerCollectionState(StingerCollectionSubsystem.StingerCollectionState.INTAKING_CORAL);
    }

    @Override
    public boolean isFinished() {
        return this.coral.confidentlyHasCoral();
    }
}
