package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;

import javax.inject.Inject;

public class IntakeCoralCommand extends BaseCommand {
    CoralScorerSubsystem coral;

    @Inject
    public IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem){
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.INTAKING);
    }

}
