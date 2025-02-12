package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.subsystems.feedback.RumbleManager;
import xbot.common.subsystems.feedback.RumbleManager_Factory;

import javax.inject.Inject;

public class IntakeCoralCommand extends BaseCommand {
    CoralScorerSubsystem coral;
    RumbleManager rumble;

    @Inject
    public IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem, RumbleManager.RumbleManagerFactory rumbleManagerFactory) {
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }


    @Override
    public void initialize() {
        coral.intake();
    }

    @Override
    public void execute(){
        if (coral.confidentlyHasCoral()){
            rumble.rumbleGamepad(100,100);
        }
    }


}
