package competition.subsystems.climber.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.climber.ClimberSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XXboxController;

import javax.inject.Inject;

public class ClimbWithJoySticksCommand extends BaseCommand {

    ClimberSubsystem climber;
    OperatorInterface oi;

    @Inject
    public ClimbWithJoySticksCommand(ClimberSubsystem climberSubsystem, OperatorInterface operatorInterface) {
        climber = climberSubsystem;
        oi = operatorInterface;
        this.addRequirements(climber);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing");
    }

    @Override
    public void execute() {
        if (oi.operatorGamepad.getXboxButton(XXboxController.XboxButton.Back).getAsBoolean()) {
            double power = oi.operatorGamepad.getRightStickY();
            climber.setPower(power);
        }

    }
}
