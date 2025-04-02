package competition.subsystems.stinger_arm.commands;

import competition.subsystems.stinger_arm.StingerArmSubsystem;
import competition.subsystems.pose.Landmarks;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;
import java.util.function.Supplier;

public class SetStingerArmTargetAngleCommand extends BaseSetpointCommand {

    StingerArmSubsystem coralArm;
    private Supplier<Landmarks.CoralLevel> angleSupplier;

    @Inject
    public SetStingerArmTargetAngleCommand(StingerArmSubsystem coralArm) {
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
