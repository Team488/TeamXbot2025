package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class EjectCoralCommand extends BaseCommand {
    CoralScorerSubsystem coralScorer;

    public boolean finished;

    @Inject
    EjectCoralCommand(CoralScorerSubsystem coralScorerSubsystem) {
        this.coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorerSubsystem);
    }

    @Override
    public void initialize() {
        this.finished = false;
    }

    @Override
    public void execute() {
        coralScorer.eject();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
