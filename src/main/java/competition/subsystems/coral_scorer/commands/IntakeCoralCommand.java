package competition.subsystems.coral_scorer.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import edu.wpi.first.wpilibj.XboxController;
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
        double triggerValue = oi.superstructureGamepad.getRawAxis(XboxController.Axis.kLeftTrigger.value);

        if (coral.confidentlyHasCoral() && triggerValue > 0.5) {
            System.out.println("Execute happened - Rumble On");
            oi.superstructureGamepad.getRumbleManager().rumbleGamepad(operatorGamePadRumblePrefrence.get(), 1);
            oi.driverGamepad.getRumbleManager().rumbleGamepad(driverGamePadRumblePrefrence.get(), 1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        oi.superstructureGamepad.getRumbleManager().rumbleGamepad(0, 0);
        oi.driverGamepad.getRumbleManager().rumbleGamepad(0.0, 0);
    }

    @Override
    public boolean isFinished() {
        return oi.superstructureGamepad.getRawAxis(XboxController.Axis.kLeftTrigger.value) <= 0.1;
    }
}
