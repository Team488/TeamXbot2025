package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IdleCoralCommand extends BaseCommand {
    final CoralScorerSubsystem coralScorer;

    @Inject
    IdleCoralCommand(CoralScorerSubsystem coralScorerSubsystem) {
        this.coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorerSubsystem);
    }

    @Override
    public void execute() {
        coralScorer.idle();
    }
}
