package competition.subsystems.drive.commands;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

public class MeasureDistanceToBranchCommand extends BaseCommand {
    PoseSubsystem pose;
    Supplier<Distance> distanceThresholdSupplier;
    Landmarks.Branch branch = Landmarks.Branch.A;
    @Inject
    public MeasureDistanceToBranchCommand(PoseSubsystem pose) {
        this.pose = pose;
    }
    @Override
    public void initialize() {
        log.info("Initializing");
    }
    @Override
    public void execute() {
        //nothing
    }
    @Override
    public boolean isFinished() {
        Pose2d targetBranchPose = Landmarks.getBranchPose(pose.getReefFaceFromAngle(), branch);
        aKitLog.record("targetBranchPose", targetBranchPose);
        aKitLog.record("distance ", pose.getCurrentPose2d().getTranslation()
                .getDistance(targetBranchPose.getTranslation()));
        aKitLog.record("branch", branch.name());

        return pose.getCurrentPose2d().getTranslation().getDistance(targetBranchPose.getTranslation())
                < distanceThresholdSupplier.get().in(Meters);

    }

    public void setDistanceThresholdSupplier(Supplier<Distance> distanceThresholdSupplier) {
        this.distanceThresholdSupplier = distanceThresholdSupplier;
    }

    public void setDistanceThreshold(Distance distanceThreshold) {
        setDistanceThresholdSupplier(() -> distanceThreshold);
    }

    public void setBranch(Landmarks.Branch branch) {
        this.branch = branch;
    }
}