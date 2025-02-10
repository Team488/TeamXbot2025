package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeUntilCoralCollectedCommand extends BaseCommand {
    CoralScorerSubsystem coral;

    @Inject
    public IntakeUntilCoralCollectedCommand(CoralScorerSubsystem coralScorerSubsystem){
        coral= coralScorerSubsystem;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.INTAKING);
    }

    @Override
    public boolean isFinished() {
        return this.coral.confidentlyHasCoral();
    }
}
