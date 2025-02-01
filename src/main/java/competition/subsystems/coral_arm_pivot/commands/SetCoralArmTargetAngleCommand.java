package competition.subsystems.coral_arm_pivot.commands;

import xbot.common.command.BaseSetpointCommand;
import xbot.common.command.SupportsSetpointLock;

public class SetCoralArmTargetAngleCommand extends BaseSetpointCommand {

    public SetCoralArmTargetAngleCommand(SupportsSetpointLock system, SupportsSetpointLock... additionalSystems) {
        super(system, additionalSystems);
    }

    public void setAngle() {

    }

    @Override
    public void initialize() {

    }
}
