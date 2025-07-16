package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeCoralCommand extends BaseCommand {
    CoralScorerSubsystem coralScorer;

    boolean finished;

    @Inject
    IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem) {
        this.coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorerSubsystem);
    }

    @Override
    public void execute() {
        coralScorer.intake();
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
