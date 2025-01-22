package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class CoralIntakeStopCommand extends BaseCommand {
    CoralScorerSubsystem coral;

    @Inject
    public CoralIntakeStopCommand(CoralScorerSubsystem coralScorerSubsystem){
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }
    @Override
    public void initialize() { coral.stop(); }
}
