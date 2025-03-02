package competition.subsystems.climber.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.climber.ClimberSubsystem;
import edu.wpi.first.units.measure.Time;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Seconds;

public class ReleaseClimberSolenoidCommand extends BaseCommand {

    ClimberSubsystem climber;

    @Inject
    public ReleaseClimberSolenoidCommand(ClimberSubsystem climberSubsystem) {
        climber = climberSubsystem;
        this.addRequirements(climber);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing");
        climber.resetPawlTimestampStart();
    }

    @Override
    public void execute() {
        climber.releaseClimberSolenoid();
    }
}
