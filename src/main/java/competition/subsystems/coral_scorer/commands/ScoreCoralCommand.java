package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;

import javax.inject.Inject;

public class ScoreCoralCommand extends BaseCommand {
    CoralScorerSubsystem coral;

    @Inject
    public ScoreCoralCommand (CoralScorerSubsystem coralScorerSubsystem){
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.SCORING);
    }

    @Override
    public void end(boolean interrupted) {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.STOPPED);
    }
}
