package competition.subsystems.stinger_collection.commands;

import competition.subsystems.stinger_collection.StingerCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeAlgaeCommand extends BaseCommand {
    StingerCollectionSubsystem coral;

    @Inject
    public IntakeAlgaeCommand (StingerCollectionSubsystem coralScorerSubsystem){
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setStingerCollectionState(StingerCollectionSubsystem.StingerCollectionState.INTAKING_ALGAE);
    }

    @Override
    public void end(boolean interrupted) {
        coral.setStingerCollectionState(StingerCollectionSubsystem.StingerCollectionState.STOPPED);
    }
}
