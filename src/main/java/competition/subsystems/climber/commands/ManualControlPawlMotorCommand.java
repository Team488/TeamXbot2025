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
    }

    @Override
    public void initialize() {
        System.out.println("Initializing");
    }

    @Override
    public void execute() {
            climber.setPawlMotorPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setPawlMotorPower(0);
    }
}
