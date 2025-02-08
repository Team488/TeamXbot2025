package competition.subsystems.drive.commands;
import competition.simulation.MapleSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseCommand;
import javax.inject.Inject;
public class SetPoseCommand extends BaseCommand {
    PoseSubsystem pose;
    @Inject
    public SetPoseCommand(PoseSubsystem pose) {
        this.pose = pose;
    }
    @Override
    public void initialize() {
        pose.setCurrentPoseInMeters(Landmarks.BlueLeftCoralStationMid);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}