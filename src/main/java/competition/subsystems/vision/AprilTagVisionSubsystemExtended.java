package competition.subsystems.vision;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.injection.electrical_contract.XCameraElectricalContract;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.vision.AprilTagVisionIO;
import xbot.common.subsystems.vision.AprilTagVisionIOFactory;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.HashMap;

@Singleton
public class AprilTagVisionSubsystemExtended extends AprilTagVisionSubsystem {
    @Inject
    public AprilTagVisionSubsystemExtended(PropertyFactory pf,
                                           AprilTagFieldLayout fieldLayout, XCameraElectricalContract contract,
                                           AprilTagVisionIOFactory visionIOFactory) {
        super(pf, fieldLayout, contract, visionIOFactory);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void refreshDataFrame() {
        super.refreshDataFrame();
    }

    public Translation2d getReefAprilTagCameraData() {
        Transform3d data = getLatestTargetObservation(0).cameraToTarget();

        return new Translation2d(data.getX(), data.getY());
    }

    public boolean reefAprilTagCameraHasCorrectTarget(int targetAprilTagID) {
        AprilTagVisionIO.TargetObservation targetObservation = getLatestTargetObservation(0);
        return targetObservation.fiducialId() == targetAprilTagID;
    }

    public int getTargetAprilTagID(Pose2d targetReefFacePose) {
        HashMap<Pose2d, Integer> hashMap = new HashMap<>();

        // Note: flipped april tag IDs across the y-midpoint of the field
        // map Red Alliance sided April Tags
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCloseLeftAlgae), 8);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCloseAlgae), 7);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCloseRightAlgae), 6);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueFarLeftAlgae), 9);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueFarAlgae), 10);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueFarRightAlgae), 11);
        }
        else { // map Blue Alliance sided April Tags
            hashMap.put(Landmarks.BlueCloseLeftAlgae, 19);
            hashMap.put(Landmarks.BlueCloseAlgae, 18);
            hashMap.put(Landmarks.BlueCloseRightAlgae, 17);
            hashMap.put(Landmarks.BlueFarLeftAlgae, 20);
            hashMap.put(Landmarks.BlueFarAlgae, 21);
            hashMap.put(Landmarks.BlueFarRightAlgae, 22);
        }
        return hashMap.get(targetReefFacePose);
    }
}
