package competition.subsystems.drive.commands;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Meters;

public class MeasureDistanceBeforeScoringCommand extends BaseCommand {
    PoseSubsystem pose;
    Distance distanceThreshold = Meters.of(5);
    @Inject
    public MeasureDistanceBeforeScoringCommand(PoseSubsystem pose) {
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
        Pose2d targetReefFacePose = Landmarks.getReefFacePose(pose.getReefFaceFromAngle());
        Distance distanceFromPoseToTarget = Meters.of(pose.getCurrentPose2d().getTranslation()
                .getDistance(targetReefFacePose.getTranslation()));

        return distanceFromPoseToTarget.isNear(Meters.of(0), distanceThreshold);
    }
}
