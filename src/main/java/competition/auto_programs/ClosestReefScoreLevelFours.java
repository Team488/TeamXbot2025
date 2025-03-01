package competition.auto_programs;

import competition.commandgroups.DriveToReefAndScoreCommandGroupFactory;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.Landmarks.ReefFace;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class ClosestReefScoreLevelFours {
    private final AprilTagFieldLayout aprilTagFieldLayout;
    final AutonomousCommandSelector autoSelector;
    PoseSubsystem pose;
    DriveToReefAndScoreCommandGroupFactory driveToReefAndScoreCommandGroupFactory;

    @Inject
    public ClosestReefScoreLevelFours(AutonomousCommandSelector autoSelector,
                                      DriveToReefAndScoreCommandGroupFactory driveToReefAndScoreCommandGroupFactory,
                                      PoseSubsystem pose, AprilTagFieldLayout aprilTagFieldLayout){
        this.autoSelector = autoSelector;
        this.pose = pose;
        this.driveToReefAndScoreCommandGroupFactory = driveToReefAndScoreCommandGroupFactory;
        this.aprilTagFieldLayout = aprilTagFieldLayout;
    }

    public BaseAutonomousSequentialCommandGroup create(Pose2d startingLocation,
                                                       Landmarks.Branch targetBranch,
                                                       Landmarks.CoralLevel targetLevel) {
        var auto = new BaseAutonomousSequentialCommandGroup(autoSelector);

        var startInFrontOfCage = pose.createSetPositionCommand(startingLocation);
        auto.addCommands(startInFrontOfCage);

        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        var reefSections = Landmarks.getAllianceReefFiducialIds(alliance);
        List<Pose2d> reefPoses = reefSections.stream()
                .map(stationFiducialId -> this.aprilTagFieldLayout.getTagPose(stationFiducialId))
                .filter(pose -> pose.isPresent())
                .flatMap(Optional::stream)
                .map(pose -> pose.toPose2d())
                .collect(Collectors.toList());

        if (reefPoses.size() == 0) {
            reefPoses = Arrays.asList(ReefFace.values()).stream()
                    .map(reefFace -> Landmarks.getReefFacePose(reefFace))
                    .collect(Collectors.toList());
        }

        var robotPose = this.pose.getCurrentPose2d();
        var targetReefFace = robotPose.nearest(reefPoses);

        auto.queueDriveClosestAndScoreMessageToAutoSelector(targetBranch, targetLevel);
        var driveAndScore = this.driveToReefAndScoreCommandGroupFactory.create(targetBranch, targetLevel);
        auto.addCommands(driveAndScore);

        return auto;
    }


}
