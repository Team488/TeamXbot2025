/**
 * Command for navigating the robot to a specified coral station section using
 * vision-based path planning. This command leverages AprilTag field layouts to
 * determine station positions and adjusts the path accordingly.
 */
package competition.subsystems.drive.commands.vision_path;

import static edu.wpi.first.units.Units.Meters;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import javax.inject.Inject;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

/**
 * Command to drive the robot to a specific coral station section.
 * Uses AprilTag vision data to determine target positions dynamically.
 */
public class PathDriveToNearestCoralStationSectionCommand
        extends PathDriveToLocationCommand {
    /**
     * Target position for the coral station section.
     */
    Pose2d targetCoralStationSection;
    Pose2d destinationPose;

    /**
     * Distance from the center of the robot to the outer bumper, in meters.
     */
    double radiusOfRobot;

    /**
     * Field layout containing AprilTag positions.
     */
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private final DriveSubsystem driveSubsystem;

    private final Distance goalThreshold = Meters.of(0.1016);

    /**
     * Constructs the PathDriveToNearestCoralStationSectionCommand.
     *
     * @param drive                             The drive subsystem.
     * @param pose                              The pose subsystem.
     * @param pf                                The property factory.
     * @param headingModuleFactory              Factory for creating heading
     *                                          modules.
     * @param electricalContract                Electrical configuration of the
     *                                          robot.
     * @param aprilTagFieldLayout               Field layout containing AprilTag
     *                                          positions.
     * @param robotAssertionManager             Assertion manager for validation.
     * @param coprocessorCommunicationSubsystem Subsystem for communicating with a
     *                                          coprocessor.
     */
    @Inject
    public PathDriveToNearestCoralStationSectionCommand(
            DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
            HeadingModule.HeadingModuleFactory headingModuleFactory,
            ElectricalContract electricalContract,
            AprilTagFieldLayout aprilTagFieldLayout,
            RobotAssertionManager robotAssertionManager,
            CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager, coprocessorCommunicationSubsystem);
        this.radiusOfRobot =
                electricalContract.getDistanceFromCenterToOuterBumperX().in(Meters);
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.driveSubsystem = drive;
    }

    /**
     * Determines the nearest coral station pose based on detected AprilTags.
     *
     * @return The estimated pose of the coral station.
     */
    private Pose2d getCoralStationPose() {
        var alliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        this.log.info("Alliance: {}", alliance);
        var coralStations = Landmarks.getAllianceCoralStationFiducialIds(alliance);
        this.log.info("Coral ids: {}", coralStations);
        List<Pose2d> stationPoses = coralStations.stream()
                .map(this.aprilTagFieldLayout::getTagPose)
                .filter(Optional::isPresent)
                .flatMap(Optional::stream)
                .map(Pose3d::toPose2d)
                .collect(Collectors.toList());

        if (stationPoses.isEmpty()) {
            stationPoses = List.of(
                    Landmarks.getCoralStationSectionPose(
                            Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID),
                    Landmarks.getCoralStationSectionPose(
                            Landmarks.CoralStation.RIGHT, Landmarks.CoralStationSection.MID));
        }

        var robotPose = this.pose.getCurrentPose2d();
        return robotPose.nearest(stationPoses);
    }

    /**
     * Initializes the command, setting the target destination based on the
     * nearest coral station section.
     */
    @Override
    public void initialize() {
        log.info("Initializing");
        targetCoralStationSection = this.getCoralStationPose();
        log.info("Picked up coral station at: {}", targetCoralStationSection);
        var deltaTranslation = new Translation2d(
                this.radiusOfRobot, targetCoralStationSection.getRotation());
        var destinationTranslation =
                targetCoralStationSection.getTranslation().plus(deltaTranslation);
        destinationPose = new Pose2d(
                destinationTranslation, targetCoralStationSection.getRotation());
        this.setTarget(destinationPose);
        this.setAdditionalArguments(
                XTableValues.AdditionalArguments.newBuilder()
                        .setAlliance(CoprocessorCommunicationSubsystem.fromAlliance(
                                DriverStation.getAlliance().orElse(
                                        DriverStation.Alliance.Blue)))
                        .build());
        this.setOptions(
                XTableValues.TraversalOptions.newBuilder()
                        .setMetersPerSecond(driveSubsystem.getDriveToWaypointsSpeed().get())
                        .setAccelerationMetersPerSecond(
                                driveSubsystem.getMaxAccelerationMetersPerSecondSquared())
                        .setFinalRotationDegrees(destinationPose.getRotation().getDegrees())
                        .build());
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        aKitLog.record("distanceToGoal",
                pose.getCurrentPose2d().getTranslation().getDistance(
                        destinationPose.getTranslation()));
        return super.isFinished()
                || (destinationPose != null
                && pose.getCurrentPose2d().getTranslation().getDistance(
                destinationPose.getTranslation())
                < goalThreshold.in(Units.Meters));
    }

    public Pose2d getTargetCoralStationSection() {
        return targetCoralStationSection;
    }
}
