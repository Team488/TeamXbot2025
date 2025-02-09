package competition.subsystems.vision;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DrivePowerCalculator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.advantage.AKitLogger;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.PIDManager;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class AlignCameraToAprilTagCalculator {
    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    final PIDManager positionalPid;

    private final int targetAprilTagID;
    private final int targetCameraID;
    public final Translation2d alignmentPointOffset;
    private final AKitLogger aKitLogger;

    public enum TagAcquisitionState {
        NeverSeen,
        LockedOn,
        Lost
    }
    private TagAcquisitionState tagAcquisitionState = TagAcquisitionState.NeverSeen;

    public static Translation2d generateAlignmentPointOffset(Distance robotCenterToOuterBumperX, CameraInfo cameraInfo,
                                                             Distance offset, boolean backwards) {
        return new Translation2d(
                (backwards ? robotCenterToOuterBumperX.times(-1) : robotCenterToOuterBumperX)
                .minus(cameraInfo.position().getMeasureX())
                .plus(backwards ? offset.times(-1) : offset),
                Meters.zero()
        );
    }

    public AlignCameraToAprilTagCalculator(AprilTagVisionSubsystemExtended vision, PIDManager positionalPid,
                                           ElectricalContract electricalContract, int targetAprilTagID,
                                           int targetCameraID, int offsetInInches, boolean backwards) {
        this.aprilTagVisionSubsystem = vision;
        this.positionalPid = positionalPid;
        this.targetAprilTagID = targetAprilTagID;
        this.targetCameraID = targetCameraID;

        this.alignmentPointOffset = generateAlignmentPointOffset(
                electricalContract.getDistanceFromCenterToOuterBumperX(),
                electricalContract.getCameraInfo()[targetCameraID],
                Inches.of(offsetInInches),
                backwards
        );

        aKitLogger = new AKitLogger("AlignCameraToAprilTagCalculator/");
        aKitLogger.record("alignmentPointOffset", this.alignmentPointOffset);
        //aKitLogger.record();
    }

    public TagAcquisitionState getTagAcquisitionState() {
        return this.tagAcquisitionState;
    }

    public Translation2d getXYPowersAlignToAprilTag(Pose2d currentPose) {
        Translation2d driveTarget = new Translation2d(0, 0);

        if (aprilTagVisionSubsystem.aprilTagCameraHasCorrectTarget(targetAprilTagID, targetCameraID)) {
            tagAcquisitionState = TagAcquisitionState.LockedOn;
            Translation2d aprilTagData = aprilTagVisionSubsystem.getAprilTagCameraData(targetCameraID);

            // This transform will always be at rotation 0, since in its own frame, the robot is always facing forward.
            Transform2d relativeGoalTransform =
                    new Transform2d(aprilTagData.minus(alignmentPointOffset), new Rotation2d());

            // Move from robot-relative frame to field frame
            driveTarget = currentPose.transformBy(relativeGoalTransform).getTranslation();
            aKitLogger.record("aprilTagData", aprilTagData);

        } else {
            if (tagAcquisitionState == TagAcquisitionState.LockedOn) {
                tagAcquisitionState = TagAcquisitionState.Lost;
            }
        }

        aKitLogger.record("driveTarget", driveTarget);
        aKitLogger.record("alignmentPointOffset", alignmentPointOffset);

        aKitLogger.record("tagAcquisitionState", tagAcquisitionState);

        return switch (tagAcquisitionState) {
            case LockedOn, Lost -> DrivePowerCalculator.getPowerToAchieveFieldPosition(
                    currentPose.getTranslation(), driveTarget, positionalPid);
            default -> new Translation2d(0, 0); // How do we get here?
        };
    }
}
