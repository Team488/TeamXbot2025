package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.Landmarks.ReefFace;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import competition.subsystems.drive.commands.DriveToBezierCurvesWithVisionCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class DriveToClosestReefSectionWithVisionCommand extends DriveToBezierCurvesWithVisionCommand {
    private final AprilTagFieldLayout aprilTagFieldLayout;

    @Inject
    DriveToClosestReefSectionWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive,
            CoprocessorCommunicationSubsystem coprocessorComms,
            PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
            RobotAssertionManager assertionManager, ElectricalContract electricalContract,
            AprilTagFieldLayout aprilTagFieldLayout) {
        super(pose, drive, coprocessorComms, pf, headingModuleFactory, assertionManager, electricalContract);
        this.aprilTagFieldLayout = aprilTagFieldLayout;
    }

    private Pose2d getClosestReefPose() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        this.log.info("Alliance: {}", alliance);
        var reefSections = Landmarks.getAllianceReefFiducialIds(alliance);
        this.log.info("reef sections: {}", reefSections);
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
        return robotPose.nearest(reefPoses);
    }

    @Override
    public void initialize() {
        if (this.setDestinationPoseForVision(this.getClosestReefPose(), false)) {
            super.initialize();
        }
    }
}
