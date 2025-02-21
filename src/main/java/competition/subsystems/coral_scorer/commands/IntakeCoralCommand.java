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
    final DoubleProperty OperatorGamePadRumblePrefrence;


    @Inject
    public IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem, OperatorInterface oi, PropertyFactory pf){
        coral = coralScorerSubsystem;
        this.oi = oi;
        this.addRequirements(coral);
        pf.setPrefix(this);
        driverGamePadRumblePrefrence = pf.createPersistentProperty("John/Driver Gamepad rumble level prefrence", .25);
        OperatorGamePadRumblePrefrence = pf.createPersistentProperty("Anthony/Opreator Gamepad rumble level prefrence", .5);


    }

    @Override
    public void initialize() {
        coral.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.INTAKING);
    }

    @Override
    public void execute(){
        if (coral.confidentlyHasCoral()){
            System.out.println("Execute happened");
            oi.superstructureGamepad.getRumbleManager().rumbleGamepad(OperatorGamePadRumblePrefrence.get(),1);
            oi.driverGamepad.getRumbleManager().rumbleGamepad(driverGamePadRumblePrefrence.get(), 1);
        } else {
            System.out.println("Stopped>?"+1);
            oi.superstructureGamepad.getRumbleManager().rumbleGamepad(0, 0);
            oi.driverGamepad.getRumbleManager().rumbleGamepad(0, 0);git 
        }
    }

}
