package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeUntilCoralEnteredFunnelCommand extends BaseCommand {
    CoralScorerSubsystem coralScorer;

    @Inject
    public IntakeUntilCoralEnteredFunnelCommand(CoralScorerSubsystem coralScorerSubsystem){
        coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorer);
    }

    @Override
    public void initialize() {
        coralScorer.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.INTAKING);
    }

    @Override
    public boolean isFinished() {
        return this.coralScorer.hasCoralEnteredFunnel();
    }
}
