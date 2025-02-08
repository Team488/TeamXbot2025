package competition.subsystems.coral_arm.commands;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.pose.Landmarks;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;

public class SetCoralArmTargetAngleCommand extends BaseSetpointCommand {

    Landmarks.CoralLevel angle;
    CoralArmSubsystem coralArm;

    @Inject
    public SetCoralArmTargetAngleCommand(CoralArmSubsystem coralArm) {
        super(coralArm);
        this.coralArm = coralArm;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        coralArm.setTargetAngle(angle);
    }

    @Override
    public void execute() {

    }

    public void setAngle(Landmarks.CoralLevel angle) { this.angle = angle; }
}
