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
    final DoubleProperty driverGamePadRumblePrefrence;
    final DoubleProperty operatorGamePadRumblePrefrence;

    @Inject
    public IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem, OperatorInterface oi, PropertyFactory pf){
        coral = coralScorerSubsystem;
        this.oi = oi;
        this.addRequirements(coral);
        pf.setPrefix(this);
        driverGamePadRumblePrefrence = pf.createPersistentProperty("John/Driver Gamepad rumble level prefrence", .25);
        operatorGamePadRumblePrefrence = pf.createPersistentProperty("Anthony/Operator Gamepad rumble level prefrence", .5);
    }

    @Override
    public void initialize() {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.INTAKING);
    }


    @Override
    public void execute(){
        System.out.println("Trigger is pressed");
        if (coral.confidentlyHasCoral()) {
            System.out.println("Execute happened - Rumble On");
            oi.operatorGamepad.getRumbleManager().rumbleGamepad(operatorGamePadRumblePrefrence.get(), 1);
            oi.driverGamepad.getRumbleManager().rumbleGamepad(driverGamePadRumblePrefrence.get(), 1);
        } else {
            System.out.println("IT DOESNT HAVE CORAL STILL");
        }
    }

}
