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
    MapleSimulator simulator;
    @Inject
    public SetPoseCommand(PoseSubsystem pose, MapleSimulator simulator) {
        this.pose = pose;
        this.simulator = simulator;
    }
    @Override
    public void initialize() {
        pose.setCurrentPoseInMeters(Landmarks.BlueLeftCoralStationMid);
        simulator.resetPosition(Landmarks.BlueLeftCoralStationMid);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}