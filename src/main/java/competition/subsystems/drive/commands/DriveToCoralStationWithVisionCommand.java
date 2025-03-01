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
    private final AprilTagFieldLayout aprilTagFieldLayout;

    @Inject
    DriveToCoralStationWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive,
            CoprocessorCommunicationSubsystem coprocessorComms,
            PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
            RobotAssertionManager assertionManager, ElectricalContract electricalContract,
            AprilTagFieldLayout aprilTagFieldLayout) {
        super(pose, drive, coprocessorComms, pf, headingModuleFactory, assertionManager, electricalContract);
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
        if (this.setDestinationPoseForVision(this.getCoralStationPose(), true)) {
            super.initialize();
        } else {
            this.log.error("Drive to closest coral station failed to initialize!");
        }
    }
}
