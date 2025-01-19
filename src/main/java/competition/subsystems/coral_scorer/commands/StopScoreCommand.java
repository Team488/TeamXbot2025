package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;

import javax.inject.Inject;

public class StopScoreCommand extends BaseCommand {
    CoralScorerSubsystem coral;

    @Inject
    public StopScoreCommand(CoralScorerSubsystem coralScorerSubsystem){
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }
    @Override
    public void initialize() { coral.stop(); }
}
