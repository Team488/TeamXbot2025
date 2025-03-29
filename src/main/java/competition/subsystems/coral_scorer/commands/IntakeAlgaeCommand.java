package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeAlgaeCommand extends BaseCommand {
    CoralScorerSubsystem coral;

    @Inject
    public IntakeAlgaeCommand (CoralScorerSubsystem coralScorerSubsystem){
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.SCORING_CORAL);
    }

    @Override
    public void end(boolean interrupted) {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.STOPPED);
    }
}
