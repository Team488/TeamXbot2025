package competition.subsystems.algae_arm.commands;

import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;

public class SetAlgaeArmSetpointToTargetPosition extends BaseSetpointCommand {

    final AlgaeArmSubsystem algaeArm;
    AlgaeArmSubsystem.AlgaeArmPositions targetPosition;

    @Inject
    public SetAlgaeArmSetpointToTargetPosition(AlgaeArmSubsystem algaeArm) {
        super(algaeArm);
        this.algaeArm = algaeArm;
        targetPosition = AlgaeArmSubsystem.AlgaeArmPositions.FullyRetracted;
    }

    public void setTargetPosition(AlgaeArmSubsystem.AlgaeArmPositions targetPosition) {
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        algaeArm.setTargetValue(targetPosition);
    }
}
