package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.vision_path.PathDriveToLocation;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.Landmarks.ReefFace;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import competition.subsystems.drive.commands.DriveToBezierCurvesWithVisionCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

import javax.inject.Inject;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class DriveToClosestReefSectionWithVisionCommand extends PathDriveToLocation {
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final DriveSubsystem driveSubsystem;

    @Inject
    DriveToClosestReefSectionWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive,
                                               CoprocessorCommunicationSubsystem coprocessorComms,
                                               PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                               RobotAssertionManager assertionManager, ElectricalContract electricalContract,
                                               AprilTagFieldLayout aprilTagFieldLayout, AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        super(drive, pose, pf, headingModuleFactory, aprilTagVisionSubsystem,
                assertionManager, coprocessorComms);
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.driveSubsystem = drive;
    }

    private Pose2d getClosestReefPose() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        this.log.info("Alliance: {}", alliance);
        var reefSections = Landmarks.getAllianceReefFiducialIds(alliance);
        this.log.info("reef sections: {}", reefSections);
        List<Pose2d> reefPoses = reefSections.stream()
                .map(this.aprilTagFieldLayout::getTagPose)
                .filter(Optional::isPresent)
                .flatMap(Optional::stream)
                .map(Pose3d::toPose2d)
                .collect(Collectors.toList());

        if (reefPoses.isEmpty()) {
            reefPoses = Arrays.stream(ReefFace.values())
                .map(Landmarks::getReefFacePose)
                .collect(Collectors.toList());
        }

        var robotPose = this.pose.getCurrentPose2d();
        return robotPose.nearest(reefPoses);
    }

    @Override
    public void initialize() {
        this.setTarget(this.getClosestReefPose());
        this.setAdditionalArguments(XTableValues.AdditionalArguments.newBuilder()
                .setAlliance(fromAlliance(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)))
                .build());
        this.setSafeInches(25);
        this.setOptions(
                XTableValues.TraversalOptions.newBuilder()
                        .setMetersPerSecond(driveSubsystem.getDriveToWaypointsSpeed().get())
                        .setAccelerationMetersPerSecond(driveSubsystem.getMaxAccelerationMetersPerSecondSquared())
                        .setFinalRotationDegrees(destinationPose.getRotation().getDegrees())
                        .build());
        super.initialize();
    }
}
