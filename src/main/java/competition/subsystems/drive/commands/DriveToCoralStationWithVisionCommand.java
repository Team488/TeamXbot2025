package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import competition.subsystems.drive.commands.DriveToBezierCurvesWithVisionCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class DriveToCoralStationWithVisionCommand extends DriveToBezierCurvesWithVisionCommand {
    Pose2d targetCoralStationSection;
    double distanceToOuterBumerInMeters;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    @Inject
    DriveToCoralStationWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive,
            CoprocessorCommunicationSubsystem coprocessorComms,
            PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
            RobotAssertionManager assertionManager, ElectricalContract electricalContract,
            AprilTagFieldLayout aprilTagFieldLayout) {
        super(pose, drive, coprocessorComms, pf, headingModuleFactory, assertionManager);
        this.distanceToOuterBumerInMeters = electricalContract.getDistanceFromCenterToOuterBumperX().in(Units.Meters);
        this.aprilTagFieldLayout = aprilTagFieldLayout;
    }

    private Pose2d getCoralStationPose() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        this.log.info("Alliance: {}", alliance);
        var coralStations = Landmarks.getAllianceCoralStationFiducialIds(alliance);
        this.log.info("coral ids: {}", coralStations);
        List<Pose2d> stationPoses = coralStations.stream()
                .map(stationFiducialId -> this.aprilTagFieldLayout.getTagPose(stationFiducialId))
                .filter(pose -> pose.isPresent())
                .flatMap(Optional::stream)
                .map(pose -> pose.toPose2d())
                .collect(Collectors.toList());

        if (stationPoses.size() == 0) {
            stationPoses = List.of(
                    Landmarks.getCoralStationSectionPose(Landmarks.CoralStation.LEFT,
                            Landmarks.CoralStationSection.MID),
                    Landmarks.getCoralStationSectionPose(Landmarks.CoralStation.RIGHT,
                            Landmarks.CoralStationSection.MID));
        }

        var robotPose = this.pose.getCurrentPose2d();
        return robotPose.nearest(stationPoses);
    }

    @Override
    public void initialize() {
        var radiusOfRobot = Math.sqrt(Math.pow(this.distanceToOuterBumerInMeters, 2.0) * 2.0);

        // TODO: Add additional logic after next test to swithc to deciding based on
        // DriverStation
        // and which coral station is clossest.
        var coralStationPose = this.getCoralStationPose();
        var deltaTranslation = new Translation2d(radiusOfRobot, coralStationPose.getRotation());
        var destinationTranslation = coralStationPose.getTranslation().plus(deltaTranslation);
        var destinationPose = new Pose2d(destinationTranslation, coralStationPose.getRotation());

        if (setTargetPoseForVision(destinationPose)) {
            super.initialize();
        }
    }
}
