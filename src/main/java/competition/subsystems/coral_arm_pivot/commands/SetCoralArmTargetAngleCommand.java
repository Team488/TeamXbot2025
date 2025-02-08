package competition.subsystems.coral_arm_pivot.commands;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.pose.Landmarks;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;

public class SetCoralArmTargetAngleCommand extends BaseSetpointCommand {

    Landmarks.CoralLevel angle;
    CoralArmPivotSubsystem coralArm;

    @Inject
    public SetCoralArmTargetAngleCommand(CoralArmPivotSubsystem coralArm) {
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
        // No-op, wait for arms to get to the target
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public void setAngle(Landmarks.CoralLevel angle) { this.angle = angle; }
}
