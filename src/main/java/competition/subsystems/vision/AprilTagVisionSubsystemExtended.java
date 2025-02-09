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
    HashMap<Pose2d, Integer> aprilTagIDHashMap = new HashMap<>();
    @Inject
    public AprilTagVisionSubsystemExtended(PropertyFactory pf,
                                           AprilTagFieldLayout fieldLayout, XCameraElectricalContract contract,
                                           AprilTagVisionIOFactory visionIOFactory) {
        super(pf, fieldLayout, contract, visionIOFactory);

        // Note: flipped april tag IDs across the y-midpoint of the field for blue alliance
        // map both blue and red alliance poses
            aprilTagIDHashMap.put(PoseSubsystem.convertBluetoRed(Landmarks.BlueCloseLeftAlgae), 8);
            aprilTagIDHashMap.put(PoseSubsystem.convertBluetoRed(Landmarks.BlueCloseAlgae), 7);
            aprilTagIDHashMap.put(PoseSubsystem.convertBluetoRed(Landmarks.BlueCloseRightAlgae), 6);
            aprilTagIDHashMap.put(PoseSubsystem.convertBluetoRed(Landmarks.BlueFarLeftAlgae), 9);
            aprilTagIDHashMap.put(PoseSubsystem.convertBluetoRed(Landmarks.BlueFarAlgae), 10);
            aprilTagIDHashMap.put(PoseSubsystem.convertBluetoRed(Landmarks.BlueFarRightAlgae), 11);
            aprilTagIDHashMap.put(Landmarks.BlueCloseLeftAlgae, 19);
            aprilTagIDHashMap.put(Landmarks.BlueCloseAlgae, 18);
            aprilTagIDHashMap.put(Landmarks.BlueCloseRightAlgae, 17);
            aprilTagIDHashMap.put(Landmarks.BlueFarLeftAlgae, 20);
            aprilTagIDHashMap.put(Landmarks.BlueFarAlgae, 21);
            aprilTagIDHashMap.put(Landmarks.BlueFarRightAlgae, 22);

    }

    public Translation2d getReefAprilTagCameraData() {
        Transform3d data = getLatestTargetObservation(0).cameraToTarget();

        return new Translation2d(data.getX(), data.getY());
    }

    public Translation2d getAprilTagCameraData(int cameraToUse) {
        Transform3d data = getLatestTargetObservation(cameraToUse).cameraToTarget();

        return new Translation2d(data.getX(), data.getY());
    }

    public boolean reefAprilTagCameraHasCorrectTarget(int targetAprilTagID) {
        AprilTagVisionIO.TargetObservation targetObservation = getLatestTargetObservation(0);
        return targetObservation.fiducialId() == targetAprilTagID;
    }

    // Assuming that John K.'s logic is all correct... (no clue how this works)
    public boolean aprilTagCameraHasCorrectTarget(int targetAprilTagID, int cameraToUse) {
        AprilTagVisionIO.TargetObservation targetObservation = getLatestTargetObservation(cameraToUse);
        return targetObservation.fiducialId() == targetAprilTagID;
    }

    public int getTargetAprilTagID(Pose2d targetReefFacePose) {
        return aprilTagIDHashMap.get(targetReefFacePose);
    }
}
