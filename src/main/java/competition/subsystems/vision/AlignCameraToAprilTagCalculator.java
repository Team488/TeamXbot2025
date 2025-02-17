package competition.subsystems.vision;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.advantage.AKitLogger;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import java.util.Optional;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class AlignCameraToAprilTagCalculator {

    public enum TagAcquisitionState {
        NeverSeen,
        LockedOn,
        Lost
    }

    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    final HeadingModule headingModule;
    final DriveSubsystem drive;
    final AKitLogger akitLog;

    final int targetAprilTagID;
    final int targetCameraID;
    final double initialHeading;
    final Translation2d alignmentPointOffset;
    final Rotation3d cameraRotation;
    Translation2d driveTarget = new Translation2d(0, 0);

    private TagAcquisitionState tagAcquisitionState = TagAcquisitionState.NeverSeen;

    public static Translation2d generateAlignmentPointOffset(Distance robotCenterToOuterBumperX, CameraInfo cameraInfo,
                                                             Distance offset, boolean isCameraBackwards) {
        return new Translation2d(
                robotCenterToOuterBumperX.times(isCameraBackwards ? -1 : 1)
                        .minus(cameraInfo.position().getMeasureX())
                        .plus(offset.times(isCameraBackwards? -1 : 1)),
                Meters.zero()
        );
    }

    public AlignCameraToAprilTagCalculator(AprilTagVisionSubsystemExtended vision, DriveSubsystem drive,
                                           ElectricalContract electricalContract, PoseSubsystem pose,
                                           HeadingModule.HeadingModuleFactory headingModuleFactory,
                                           int targetAprilTagID, int targetCameraID, Distance offset,
                                           boolean isCameraBackwards) {
        this.aprilTagVisionSubsystem = vision;
        this.targetAprilTagID = targetAprilTagID;
        this.targetCameraID = targetCameraID;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.drive = drive;
        drive.getPositionalPid().reset();

        this.initialHeading = pose.getCurrentPose2d().getRotation().getDegrees();

        CameraInfo cameraInfo = electricalContract.getCameraInfo()[targetCameraID];
        this.cameraRotation = cameraInfo.position().getRotation();

        this.alignmentPointOffset = generateAlignmentPointOffset(
                electricalContract.getDistanceFromCenterToOuterBumperX(),
                cameraInfo,
                offset,
                isCameraBackwards
        );

        this.akitLog = new AKitLogger("AlignCameraToAprilTagCalculator/");
    }

    public Pose2d getXYPowersAlignToAprilTag(Pose2d currentPose) {

        if (aprilTagVisionSubsystem.doesCameraBestObservationHaveAprilTagId(targetCameraID, targetAprilTagID)) {
            tagAcquisitionState = TagAcquisitionState.LockedOn;
            Translation2d aprilTagData = aprilTagVisionSubsystem.getRobotRelativeLocationOfBestDetectedAprilTag(targetCameraID);

            akitLog.record("AprilTagData", aprilTagData);

            // This transform will always be at rotation 0, since in its own frame, the robot is always facing forward.
            Transform2d relativeGoalTransform = new Transform2d(
                    aprilTagData.minus(alignmentPointOffset),
                    new Rotation2d()
            );

            // Move from robot-relative frame to field frame
            driveTarget = currentPose.transformBy(relativeGoalTransform).getTranslation();

        } else {
            if (tagAcquisitionState == TagAcquisitionState.LockedOn) {
                tagAcquisitionState = TagAcquisitionState.Lost;
            }
        }

        return switch (tagAcquisitionState) {
            case LockedOn, Lost -> {
                // Flip our desiredRotation if backwards
                Optional<Pose3d> aprilTagPose = aprilTagVisionSubsystem.getAprilTagFieldOrientedPose(targetAprilTagID);
                double desiredRotation = aprilTagPose.map(
                        (tag) -> Math.PI + tag.getRotation().getZ() - cameraRotation.getZ()
                ).orElse(initialHeading);
                akitLog.record("desiredRotation", desiredRotation);

                yield new Pose2d(
                        drive.getPowerToAchieveFieldPosition(currentPose.getTranslation(), driveTarget),
                        Rotation2d.fromDegrees(headingModule.calculateHeadingPower(Math.toDegrees(desiredRotation))) // QUESTION: IS THIS HOW I DO IT?
                );
            }
            default -> new Pose2d(0, 0, new Rotation2d()); // How do we get here?
        };
    }

    public boolean recommendIsFinished() {
        return drive.getPositionalPid().isOnTarget()
                && drive.getRotateToHeadingPid().isOnTarget()
                && tagAcquisitionState != TagAcquisitionState.NeverSeen;
    }
}
