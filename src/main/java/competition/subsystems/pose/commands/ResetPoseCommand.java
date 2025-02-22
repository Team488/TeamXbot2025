package competition.subsystems.pose.commands;

import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ResetPoseCommand extends BaseCommand {

    final PoseSubsystem pose;
    Pose2d poseTarget = new Pose2d(0, 0, new Rotation2d(0));

    @Inject
    public ResetPoseCommand(PoseSubsystem pose) {
        this.pose = pose;
    }

    public void setPoseTarget(Pose2d pose) {
        this.poseTarget = pose;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        pose.setCurrentPoseInMeters(poseTarget);
    }

    @Override
    public void execute() {
        // No-code
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
