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
    final PoseSubsystem pose;
    final AKitLogger akitLog;

    final int targetAprilTagID;
    final int targetCameraID;
    final double initialHeading;
    final Translation2d alignmentPointOffset;
    final Rotation3d cameraRotation;
    Translation2d targetLocationOnField = new Translation2d(0, 0);

    final double distanceForExactAlignment = 1;

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
        this.pose = pose;
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
        Translation2d targetLocationOnField = new Translation2d(0, 0);
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
            targetLocationOnField = currentPose.transformBy(relativeGoalTransform).getTranslation();

        } else {
            if (tagAcquisitionState == TagAcquisitionState.LockedOn) {
                tagAcquisitionState = TagAcquisitionState.Lost;
            }
        }

        return switch (tagAcquisitionState) {
            case LockedOn, Lost -> {
                // Flip our desiredRotation if backwards
                Optional<Pose3d> aprilTagPose = aprilTagVisionSubsystem.getAprilTagFieldOrientedPose(targetAprilTagID);
                double desiredHeading = initialHeading; // Or should we use current heading to avoid possible oscillation?
                if (aprilTagPose.isPresent()) {
                    Translation2d aprilTagPosition = aprilTagPose.get().getTranslation().toTranslation2d();

                    if (isCloseEnoughForExactAlignment(aprilTagPosition)) {
                        desiredHeading = Math.PI + aprilTagPose.get().getRotation().getZ() - cameraRotation.getZ();
                    } else {
                        Translation2d currentTranslation = pose.getCurrentPose2d().getTranslation();
                        desiredHeading = currentTranslation.minus(aprilTagPosition).getAngle().getRadians() + Math.PI;
                    }
                }

                yield new Pose2d(
                        drive.getPowerToAchieveFieldPosition(currentPose.getTranslation(), targetLocationOnField),
                        Rotation2d.fromDegrees(headingModule.calculateHeadingPower(Math.toDegrees(desiredHeading)))
                );
            }
            default -> new Pose2d(0, 0, new Rotation2d()); // How do we get here?
        };
    }

    private boolean isCloseEnoughForExactAlignment(Translation2d targetPosition) {
        return targetPosition.getDistance(pose.getCurrentPose2d().getTranslation())
                < (distanceForExactAlignment + alignmentPointOffset.getNorm());
    }

    public boolean recommendIsFinished() {
        return drive.getPositionalPid().isOnTarget()
                && drive.getRotateToHeadingPid().isOnTarget()
                && tagAcquisitionState != TagAcquisitionState.NeverSeen;
    }
}
