package competition.subsystems.humanLoadRamp.commands;

import competition.subsystems.humanLoadRamp.HumanLoadRampSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class HumanLoadRampExtendCommand extends BaseCommand {
    HumanLoadRampSubsystem ramp;

    @Inject
    public HumanLoadRampExtendCommand (HumanLoadRampSubsystem humanLoadRampSubsystem){
        ramp = humanLoadRampSubsystem;
        this.addRequirements(ramp);
    }
    public void initialize(){
        ramp.extend();
    }
}
