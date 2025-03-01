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

    //timestamp in seconds
    final Time pawlTimestampStart = XTimer.getFPGATimestampTime();
    final Time pawlTimestampEnd = pawlTimestampStart.plus(Seconds.of(1.5));
    final Time pawlTimestampCooldown = pawlTimestampEnd.plus(Seconds.of(6));

    @Inject
    public ReleaseClimberSolenoidCommand(ClimberSubsystem climberSubsystem) {
        climber = climberSubsystem;
        this.addRequirements(climber);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing");
    }

    @Override
    public void execute() {
        if (XTimer.getFPGATimestampTime().lt(pawlTimestampEnd)){
            climber.setPawlMotorPower(1);
        }

        //prevents motor from spinning while on cooldown
        if (XTimer.getFPGATimestampTime().lt(pawlTimestampCooldown) && XTimer.getFPGATimestampTime().gt(pawlTimestampEnd)){
            climber.setPawlMotorPower(0);
        }
    }
}
