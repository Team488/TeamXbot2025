package competition.subsystems.coral_arm.commands;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.pose.Landmarks;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;
import java.util.function.Supplier;

public class SetCoralArmTargetAngleCommand extends BaseSetpointCommand {

    CoralArmSubsystem coralArm;
    private Supplier<Landmarks.CoralLevel> angleSupplier;

    @Inject
    public SetCoralArmTargetAngleCommand(CoralArmSubsystem coralArm) {
        super(coralArm);
        this.coralArm = coralArm;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        coralArm.setTargetAngle(this.angleSupplier.get());
    }

    @Override
    public void execute() {
        // No-op, wait for arms to get to the target
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public void setAngle(Landmarks.CoralLevel angle) {
        setAngleSupplier(() -> angle);
    }
    public void setAngleSupplier(Supplier<Landmarks.CoralLevel> angleSupplier) {
        this.angleSupplier = angleSupplier;
    }
}
