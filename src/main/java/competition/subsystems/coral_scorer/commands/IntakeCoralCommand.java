package competition.subsystems.coral_scorer.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class IntakeCoralCommand extends BaseCommand {
    final CoralScorerSubsystem coral;
    final OperatorInterface oi;


    @Inject
    public IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem, OperatorInterface oi, PropertyFactory pf){
        coral = coralScorerSubsystem;
        this.oi = oi;
        this.addRequirements(coral);

    }

    @Override
    public void initialize() {
        this.log.info("Intialize");
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.INTAKING);
    }


    @Override
    public void execute(){
        if (coral.confidentlyHasCoral()) {
            oi.getOperatorGamepadRumbleIntensitiy();
            oi.getDriverGamepadRumbleIntensitity();
        }
    }

}
