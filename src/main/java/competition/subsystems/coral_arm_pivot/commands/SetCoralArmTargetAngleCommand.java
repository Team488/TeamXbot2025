package competition.subsystems.coral_arm_pivot.commands;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;

public class SetCoralArmTargetAngleCommand extends BaseSetpointCommand {

    CoralArmPivotSubsystem.ArmGoals angle;
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

    }

    public void setAngle(CoralArmPivotSubsystem.ArmGoals angle) { this.angle = angle; }
}
