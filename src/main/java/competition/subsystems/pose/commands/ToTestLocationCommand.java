package competition.subsystems.pose.commands;

import competition.simulation.MapleSimulator;
import competition.simulation.Simulator;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;
import java.util.Map;

public class ToTestLocationCommand extends BaseCommand {

    PoseSubsystem pose;
    MapleSimulator simulator;

    @Inject
    public ToTestLocationCommand(PoseSubsystem pose, MapleSimulator simulator) {
        this.pose = pose;
        this.simulator = simulator;
    }

    @Override
    public void initialize() {
        pose.setCurrentPoseInMeters(new Pose2d(2, 2, new Rotation2d(0)));
        simulator.resetPosition(new Pose2d(2, 2, new Rotation2d(0)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
