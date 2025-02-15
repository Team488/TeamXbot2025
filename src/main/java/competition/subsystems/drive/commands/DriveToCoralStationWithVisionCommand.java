package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import competition.subsystems.drive.commands.DriveToWaypointsWithVisionCommand;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class DriveToCoralStationWithVisionCommand extends DriveToWaypointsWithVisionCommand {
    Pose2d targetCoralStationSection;
    double distanceToOuterBumerInMeters;

    @Inject
    DriveToCoralStationWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive, CoprocessorCommunicationSubsystem coprocessorComms,
                                         PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                         RobotAssertionManager assertionManager, ElectricalContract electricalContract) {
        super(pose, drive, coprocessorComms, pf, headingModuleFactory, assertionManager);
        this.distanceToOuterBumerInMeters = electricalContract.getDistanceFromCenterToOuterBumperX().in(Units.Meters);
    }

    @Override
    public void initialize() {
        var radiusOfRobot = Math.sqrt(Math.pow(this.distanceToOuterBumerInMeters, 2.0) * 2.0);
        var coralStationPose = Landmarks.getCoralStationSectionPose(Landmarks.CoralStation.LEFT,
                Landmarks.CoralStationSection.MID);
        var deltaTranslation = new Translation2d(radiusOfRobot, coralStationPose.getRotation());
        var destinationTranslation = coralStationPose.getTranslation().plus(deltaTranslation);
        var destinationPose = new Pose2d(destinationTranslation, coralStationPose.getRotation());

        if (setTargetPoseForVision(destinationPose)) {
            super.initialize();
        }
    }
}
