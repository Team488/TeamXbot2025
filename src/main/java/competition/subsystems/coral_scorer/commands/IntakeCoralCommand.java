package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeCoralCommand extends BaseCommand {
    final CoralScorerSubsystem coralScorer;

    @Inject
    IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem) {
        this.coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorerSubsystem);
    }

    @Override
    public void execute() {
        coralScorer.intake();
    }
}
