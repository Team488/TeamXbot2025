package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class DefaultCoralCommand extends BaseCommand {
    final CoralScorerSubsystem coralScorer;

    @Inject
    DefaultCoralCommand(CoralScorerSubsystem coralScorerSubsystem) {
        this.coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorerSubsystem);
    }

    @Override
    public void execute() {
        if (coralScorer.hasCoral()) {
            coralScorer.holdCoral();
        } else {
            coralScorer.idle();
        }
    }
}
