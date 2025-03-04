package competition.subsystems.drive.commands;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

public class MeasureDistanceToFaceCommand extends BaseCommand {
    PoseSubsystem pose;
    Supplier<Distance> distanceThresholdSupplier;
    Landmarks.ReefFace face = Landmarks.ReefFace.CLOSE;
    @Inject
    public MeasureDistanceToFaceCommand(PoseSubsystem pose) {
        this.pose = pose;
    }
    @Override
    public void initialize() {
        log.info("Initializing");
    }
    @Override
    public void execute() {
        // nothing
    }
    @Override
    public boolean isFinished() {
        Pose2d targetReefFacePose = Landmarks.getReefFacePose(pose.getReefFaceFromAngle());
        aKitLog.record("targetReefFacePose", targetReefFacePose);
        aKitLog.record("distance ", pose.getCurrentPose2d().getTranslation()
                .getDistance(targetReefFacePose.getTranslation()));
        aKitLog.record("face", face.name());

        return pose.getCurrentPose2d().getTranslation().getDistance(targetReefFacePose.getTranslation())
                < distanceThresholdSupplier.get().in(Meters);

    }

    public void setDistanceThresholdSupplier(Supplier<Distance> distanceThresholdSupplier) {
        this.distanceThresholdSupplier = distanceThresholdSupplier;
    }

    public void setDistanceThreshold(Distance distanceThreshold) {
        setDistanceThresholdSupplier(() -> distanceThreshold);
    }
}