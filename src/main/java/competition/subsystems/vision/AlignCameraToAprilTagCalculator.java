package competition.subsystems.vision;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DrivePowerCalculator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.PIDManager;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class AlignCameraToAprilTagCalculator {
    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    final PIDManager positionalPid;
    final ElectricalContract electricalContract;

    private final int targetAprilTagID;
    private final int targetCameraID;
    private final Translation2d alignmentPointOffset;

    public enum TagAcquisitionState {
        NeverSeen,
        LockedOn,
        Lost
    }
    private TagAcquisitionState tagAcquisitionState = TagAcquisitionState.NeverSeen;

    public AlignCameraToAprilTagCalculator(AprilTagVisionSubsystemExtended vision,
                                           ElectricalContract electricalContract, PIDManager positionalPid,
                                           int targetAprilTagID, int targetCameraID, int offset, boolean backwards) {
        this.aprilTagVisionSubsystem = vision;
        this.positionalPid = positionalPid;
        this.electricalContract = electricalContract;
        this.targetAprilTagID = targetAprilTagID;
        this.targetCameraID = targetCameraID;

        CameraInfo[] cameraInfos = electricalContract.getCameraInfo();

        // We only care about X as for Y we want to center to the tag so 0.
        Distance distanceFromCenterToOuterBumperX = electricalContract.getDistanceFromCenterToOuterBumperX();
        if (backwards) {
            distanceFromCenterToOuterBumperX.times(-1);
        }
        this.alignmentPointOffset = new Translation2d(
                distanceFromCenterToOuterBumperX.minus(cameraInfos[targetCameraID].position().getMeasureX())
                        .plus(Inches.of(backwards ? -offset : offset)),
                Meters.zero()
        );
    }

    public TagAcquisitionState getTagAcquisitionState() {
        return this.tagAcquisitionState;
    }

    public Translation2d getXYPowersAlignToAprilTag(Pose2d currentPose) {
        Translation2d driveTarget = new Translation2d(0, 0);

        if (aprilTagVisionSubsystem.aprilTagCameraHasCorrectTarget(targetAprilTagID, targetCameraID)) {
            tagAcquisitionState = TagAcquisitionState.LockedOn;
            Translation2d aprilTagData = aprilTagVisionSubsystem.getReefAprilTagCameraData();

            // This transform will always be at rotation 0, since in its own frame, the robot is always facing forward.
            Transform2d relativeGoalTransform =
                    new Transform2d(aprilTagData.minus(alignmentPointOffset), new Rotation2d());

            // Move from robot-relative frame to field frame
            driveTarget = currentPose.transformBy(relativeGoalTransform).getTranslation();

        } else {
            if (tagAcquisitionState == TagAcquisitionState.LockedOn) {
                tagAcquisitionState = TagAcquisitionState.Lost;
            }
        }

        return switch (tagAcquisitionState) {
            case LockedOn, Lost -> DrivePowerCalculator.getPowerToAchieveFieldPosition(
                    currentPose.getTranslation(), driveTarget, positionalPid);
            default -> new Translation2d(0, 0); // How do we get here?
        };
    }
}
