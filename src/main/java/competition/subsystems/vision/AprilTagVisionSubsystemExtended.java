package competition.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.injection.electrical_contract.XCameraElectricalContract;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.vision.AprilTagVisionIO;
import xbot.common.subsystems.vision.AprilTagVisionIOFactory;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

import javax.inject.Inject;
import javax.inject.Singleton;

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
}
