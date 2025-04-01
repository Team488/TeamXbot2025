package competition.subsystems.drive.commands.vision_path;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DistanceProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.vision.AprilTagVisionIO;

import javax.inject.Inject;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;

public class PathDriveToReefFaceUntilAprilTagDetection
        extends PathDriveToReefFaceCommand {
    int aprilTagID;

    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    private Cameras camera = Cameras.FRONT_LEFT_CAMERA;

    private final DistanceProperty distanceProperty;
    @Inject
    PathDriveToReefFaceUntilAprilTagDetection(PoseSubsystem pose,
                                              DriveSubsystem drive, CoprocessorCommunicationSubsystem coprocessorComms,
                                              PropertyFactory pf,
                                              HeadingModule.HeadingModuleFactory headingModuleFactory,
                                              RobotAssertionManager assertionManager,
                                              ElectricalContract electricalContract,
                                              AprilTagFieldLayout aprilTagFieldLayout,
                                              AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        super(pose, drive, coprocessorComms, pf, headingModuleFactory,
                assertionManager, electricalContract, aprilTagFieldLayout);
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        pf.setPrefix("PathDriveToReefFaceUntilAprilTagDetection/");
        this.distanceProperty = pf.createPersistentProperty("AprilTagDistanceThreshold", Meters.of(1.2));

    }

    @Override
    public PathDriveToReefFaceCommand setBranch(Landmarks.Branch branch) {
        if (branch.equals(Landmarks.Branch.A)) {
            this.camera = Cameras.FRONT_LEFT_CAMERA;
        } else {
            this.camera = Cameras.FRONT_RIGHT_CAMERA;
        }
        return super.setBranch(branch);
    }

    @Override
    public PathDriveToReefFaceCommand setReefFace(Landmarks.ReefFace reefFace) {
        DriverStation.Alliance alliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        this.aprilTagID = alliance.equals(DriverStation.Alliance.Blue)
                ? reefFace.getBlueAprilTagID()
                : reefFace.getRedAprilTagID();
        return super.setReefFace(reefFace);
    }

    @Override
    public boolean isFinished() {
        AprilTagVisionIO.TargetObservation targetObservation =
                aprilTagVisionSubsystem.getLatestTargetObservation(camera.getIndex());

        boolean closeEnough = false;
        if (targetObservation.fiducialId() == aprilTagID) {
            Transform3d transform3d = targetObservation.cameraToTarget();
            double distance = transform3d.getTranslation().getNorm();
            closeEnough = distance <= distanceProperty.get().in(Meters);
        }

        return super.isFinished() || closeEnough;

    }
}
