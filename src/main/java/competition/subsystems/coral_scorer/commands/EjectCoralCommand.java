package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class EjectCoralCommand extends BaseCommand {
    final CoralScorerSubsystem coralScorer;

    @Inject
    EjectCoralCommand(CoralScorerSubsystem coralScorerSubsystem) {
        this.coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorerSubsystem);
    }

    @Override
    public void execute() {
        coralScorer.eject();
    }
}
