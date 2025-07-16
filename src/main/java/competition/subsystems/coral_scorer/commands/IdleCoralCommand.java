package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IdleCoralCommand extends BaseCommand {
    CoralScorerSubsystem coralScorer;

    public boolean finished;

    @Inject
    IdleCoralCommand(CoralScorerSubsystem coralScorerSubsystem) {
        this.coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorerSubsystem);
    }

    @Override
    public void execute() {
        coralScorer.idle();
        finished = true;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
