package competition.subsystems.coral_scorer.commands;

import competition.operator_interface.OperatorCommandMap;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.subsystems.feedback.RumbleManager;
import xbot.common.subsystems.feedback.RumbleManager_Factory;

import javax.inject.Inject;

public class IntakeCoralCommand extends BaseCommand {
    CoralScorerSubsystem coral;
    RumbleManager rumble;
    OperatorInterface oi;

    @Inject
    public IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem, RumbleManager.RumbleManagerFactory rumbleManagerFactory) {
        coral = coralScorerSubsystem;
        this.addRequirements(coral);
    }


    @Override
    public void initialize() {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.INTAKING);
    }

    @Override
    public void execute(){
        if (coral.confidentlyHasCoral()){
            oi.operatorGamepad.getRumbleManager().rumbleGamepad(10000,1000000);
        }
    }


}
