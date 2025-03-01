package competition.subsystems.climber.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.climber.ClimberSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XXboxController;

import javax.inject.Inject;

public class ManualControlPawlMotorCommand extends BaseCommand {

    ClimberSubsystem climber;
    OperatorInterface oi;
    @Inject
    public ManualControlPawlMotorCommand(ClimberSubsystem climber, OperatorInterface oi){
        this.climber = climber;
        this.oi = oi;
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
            climber.setPawlMotorPower(power);
        }
    }
}
