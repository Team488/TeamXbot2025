package competition.subsystems.humanloadramp.commands;

import competition.subsystems.humanloadramp.HumanLoadRampSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class HumanLoadRampRetractCommand extends BaseCommand {
    HumanLoadRampSubsystem ramp;

    @Inject
    public HumanLoadRampRetractCommand (HumanLoadRampSubsystem humanLoadRampSubsystem){
        ramp = humanLoadRampSubsystem;
        this.addRequirements(ramp);

    }
    public void initialize(){
        ramp.retract();
    }
}
