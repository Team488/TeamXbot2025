package competition.subsystems.drive.logic;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.sensors.XTimer;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.vision.AprilTagVisionIO;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class AlignCameraToAprilTagCalculator {

    public enum TagAcquisitionState {
        NeverSeen,
        LockedOn,
        Lost
    }

    public enum Activity {
        Searching,
        ApproachWhileCentering,
        TerminalApproach,
        Shove,
        Complete
    }

    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    final HeadingModule headingModule;
    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final AKitLogger akitLog;
    final ElectricalContract electricalContract;
    final ReefCoordinateGenerator reefCoordinateGenerator;

    int targetAprilTagID;
    int targetCameraID;
    double initialHeading;
    Translation2d alignmentPointOffset;
    Rotation3d cameraRotation;

    private boolean isCameraBackwards = false;
    private TagAcquisitionState tagAcquisitionState = TagAcquisitionState.NeverSeen;
    private Activity activity = Activity.Searching;
    Translation2d targetLocationOnField = new Translation2d(0, 0);

    final DoubleProperty interstitialDistance;
    final DoubleProperty distanceFromInterstitialToAdvance;
    final DoubleProperty approachSpeedFactor;
    final DoubleProperty distanceToStartShoving;
    final DoubleProperty shovePower;
    final DoubleProperty shoveDuration;
    final DoubleProperty maxTagAmbiguity;


    double shoveStartTime = 0;

    public static Translation2d generateAlignmentPointOffset(Distance robotCenterToOuterBumperX, CameraInfo cameraInfo,
                                                             Distance offset, boolean isCameraBackwards) {
        return new Translation2d(
                robotCenterToOuterBumperX.times(isCameraBackwards ? -1 : 1)
                        .minus(cameraInfo.position().getMeasureX())
                        .plus(offset.times(isCameraBackwards? -1 : 1)),
                Meters.zero()
        );
    }

    String prefix = "AlignCameraToAprilTagCalculator/";

    @AssistedFactory
    public abstract static class AlignCameraToAprilTagCalculatorFactory {
        public abstract AlignCameraToAprilTagCalculator create();
    }

    @AssistedInject
    public AlignCameraToAprilTagCalculator(AprilTagVisionSubsystemExtended vision, DriveSubsystem drive,
                                           ElectricalContract electricalContract, PoseSubsystem pose,
                                           HeadingModule.HeadingModuleFactory headingModuleFactory, ReefCoordinateGenerator reefCoordinateGenerator,
                                           PropertyFactory pf) {
        this.aprilTagVisionSubsystem = vision;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.drive = drive;
        this.pose = pose;
        this.electricalContract = electricalContract;
        this.reefCoordinateGenerator = reefCoordinateGenerator;
        this.akitLog = new AKitLogger(prefix);

        pf.setPrefix(prefix);
        interstitialDistance = pf.createPersistentProperty("InterstitialDistance-m", 2.0);
        distanceFromInterstitialToAdvance = pf.createPersistentProperty("DistanceFromInterstitialToAdvance-m", 0.2);
        approachSpeedFactor = pf.createPersistentProperty("ApproachSpeedFactor", 0.75);
        distanceToStartShoving = pf.createPersistentProperty("DistanceToStartShoving-m", 0.0762); // 3 inches
        shovePower = pf.createPersistentProperty("ShovePower", 0.25);
        shoveDuration = pf.createPersistentProperty("ShoveDuration-s", 0.5);
        maxTagAmbiguity = pf.createPersistentProperty("MaxTagAmbiguity", 0.5);

        reset();
    }

    private void reset() {
        drive.getPositionalPid().reset();
        tagAcquisitionState = TagAcquisitionState.NeverSeen;
        activity = Activity.Searching;
    }

    public void configureAndReset(int targetAprilTagID, int targetCameraID, Distance offset,
                                  boolean isCameraBackwards) {
        reset();

        this.targetAprilTagID = targetAprilTagID;
        this.targetCameraID = targetCameraID;
        this.isCameraBackwards = isCameraBackwards;

        this.initialHeading = pose.getCurrentPose2d().getRotation().getDegrees();
        this.isCameraBackwards = isCameraBackwards;

        CameraInfo cameraInfo = electricalContract.getCameraInfo()[targetCameraID];
        this.cameraRotation = cameraInfo.position().getRotation();

        this.alignmentPointOffset = generateAlignmentPointOffset(
                electricalContract.getDistanceFromCenterToOuterBumperX(),
                cameraInfo,
                offset,
                isCameraBackwards
        );

        // Now for some other one-time calculations about the tag itself
        Optional<Pose3d> aprilTagPose = aprilTagVisionSubsystem.getAprilTagFieldOrientedPose(targetAprilTagID);
        aprilTagPositionInGlobalFieldCoordinates = aprilTagPose.map((p) -> p.getTranslation().toTranslation2d()).orElse(new Translation2d(0, 0));
        aprilTagZRotationRadians = aprilTagPose.map((p) -> p.getRotation().getZ()).orElse(0.0);
        idealFinalHeadingDegrees = Radians.of(Math.PI + aprilTagZRotationRadians - cameraRotation.getZ()).in(Degrees);

        // We can also calculate our ideal interstitial point, since that's a one-time calculation
        // First, we need to see if we are using the left or right camera.
        boolean isLeft = cameraInfo.friendlyName().toLowerCase().contains("left");
        // When using the left camera, that means we are aligning on the right branch, aka, the B branch.
        // Now we have enough information to find the interstitial point.

        interstitialPoint = new Pose2d();
        Landmarks.FieldElementType elementType = Landmarks.getFieldElementTypeForAprilTag(targetAprilTagID);

        if (elementType == Landmarks.FieldElementType.REEF_FACE) {
            interstitialPoint = reefCoordinateGenerator.getPoseRelativeToReefFaceAndBranch(
                            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue),
                            Landmarks.getReefFaceFromTagId(targetAprilTagID),
                            isLeft ? Landmarks.Branch.B : Landmarks.Branch.A,
                            Meters.of(interstitialDistance.get()),
                            Meters.zero()
                    );
        } else if (elementType == Landmarks.FieldElementType.CORAL_STATION) {
            // TODO - figure this out once we have a rear-facing camera
            int foo = 0;
        }

        akitLog.record("InterstitialPoint", interstitialPoint);
    }

    Translation2d aprilTagPositionInGlobalFieldCoordinates;
    double aprilTagZRotationRadians;
    double idealFinalHeadingDegrees;
    Pose2d interstitialPoint;

    public AlignCameraToAprilTagAdvice getXYPowersAlignToAprilTag(Pose2d currentPose) {

        // First, let's get any evergreen information we will need in almost all state machines.
        // Mostly, this is about where we should be pointing - and we generally point at the tag unless we are fairly close.
        Optional<AprilTagVisionIO.TargetObservation> targetObservation = aprilTagVisionSubsystem.getTargetObservation(targetCameraID, targetAprilTagID);
        boolean doWeSeeOurTargetTag = targetObservation.isPresent() && targetObservation.get().ambiguity() < maxTagAmbiguity.get();
        Translation2d currentTranslation = pose.getCurrentPose2d().getTranslation();
        double headingToPointAtAprilTag = Radians.of(
                currentTranslation.minus(aprilTagPositionInGlobalFieldCoordinates).getAngle().getRadians() + Math.PI
        ).plus(Radians.of(isCameraBackwards ? Math.PI : 0)).in(Degrees);

        // Eventually we need to return these - they will likely be mutated by later steps.
        XYPair driveIntent = new XYPair(0, 0);
        double rotationIntent = 0;

        // To allow for quick drop-through, we will have the first step of the state machine be an "if" statement
        // so that if we see the tag we can jump right into the meat of the machine.
        if (activity == Activity.Searching ) {
            // We see the tag. Begin the approach.
            if (doWeSeeOurTargetTag) {
                activity = Activity.ApproachWhileCentering;
            } else {
                // We don't see the tag, so point at where it might be. Nothing else can be done,
                // so tell the caller to point at the april tag. Maybe we will see it in future loops.
                return new AlignCameraToAprilTagAdvice(
                        driveIntent,
                        headingModule.calculateHeadingPower(headingToPointAtAprilTag),
                        tagAcquisitionState, activity);
            }
        }

        switch (activity) {
            case ApproachWhileCentering:
                // First, let's try and update our final position with any camera data, in case we lose it later.
                if (doWeSeeOurTargetTag) {
                    tagAcquisitionState = TagAcquisitionState.LockedOn;
                    updateFinalTargetState(currentPose);
                } else {
                    tagAcquisitionState = TagAcquisitionState.Lost;
                }

                // We have seen the tag at least once. We will approach it in stages.
                // First, we want to approach a point that's a bit far away from the tag, but straight in line with it.
                // So, we head to the interstitial point while pointing at the april tag

                // We want to hit the interstitial point at speed, so rather than using some kind of PID, we just
                // drive straight at it with a commanded velocity. For that, we need to get a vector in the direction
                // of the interstitial point, then normalize it and multiply by the speed we want to go.
                // This should work since the drive subsystem takes in "drive intents" from 0-1 representing 0-100% velocity,
                // so this will automatically scale with the robot's max speed.
                var vectorTowardsInterstitial = interstitialPoint.getTranslation().minus(currentPose.getTranslation());
                var normalizedVector = vectorTowardsInterstitial.div(vectorTowardsInterstitial.getNorm());
                driveIntent = new XYPair(normalizedVector.getX(), normalizedVector.getY()).scale(approachSpeedFactor.get());
                rotationIntent = headingModule.calculateHeadingPower(headingToPointAtAprilTag);

                // Finally, a check to see if we're quite close and should advance to the next state.
                if (currentPose.getTranslation().getDistance(interstitialPoint.getTranslation()) < distanceFromInterstitialToAdvance.get()) {
                    activity = Activity.TerminalApproach;
                }
                break;
            case TerminalApproach:
                // We are effectively at the interstitial point. We now lock our heading to the final heading and try to approach
                // the final point with some caution, meaning we will use the Drive PID to decelerate.

                // As before, first we see if we can get any fresh data:
                if (doWeSeeOurTargetTag) {
                    tagAcquisitionState = TagAcquisitionState.LockedOn;
                    updateFinalTargetState(currentPose);
                } else {
                    tagAcquisitionState = TagAcquisitionState.Lost;
                }

                // Now, drive to that final point with locked-on heading.
                var powers = drive.getPowerToAchieveFieldPosition(currentPose.getTranslation(), targetLocationOnField);
                driveIntent = new XYPair(powers.getX(), powers.getY());
                rotationIntent = headingModule.calculateHeadingPower(idealFinalHeadingDegrees);

                // If we're quite close to the final point, advance to shoving into the reef or coral station.
                if (currentPose.getTranslation().getDistance(targetLocationOnField) < distanceToStartShoving.get()) {
                    activity = Activity.Shove;
                    shoveStartTime = XTimer.getFPGATimestamp();
                }
                break;
            case Shove:
                // We are so very close to our destination, but it's very hard to get perfect alignment -- the PID
                // will bring us close, but error in the dive might mean we are off by a few inches. We are also
                // so close to the april tag that it's no longer guaranteed to be in view. So, we will just try and
                // drive straight into the reef.

                // We know the ideal angle the robot will be pointing at, so we can quickly construct a shove vector
                // in that direction.
                driveIntent = XYPair.fromPolar(idealFinalHeadingDegrees, shovePower.get());
                rotationIntent = headingModule.calculateHeadingPower(idealFinalHeadingDegrees);

                // If we've been shoving for a while, we're done.
                if (XTimer.getFPGATimestamp() - shoveStartTime > shoveDuration.get()) {
                    activity = Activity.Complete;
                }
                break;
            case Complete:
            default:
                // We're done! We don't need to do anything.
                break;
        }

        akitLog.record("Activity", activity);
        akitLog.record("TagAcquisitionState", tagAcquisitionState);

        return new AlignCameraToAprilTagAdvice(driveIntent, rotationIntent, tagAcquisitionState, activity);
    }

    private void updateFinalTargetState(Pose2d currentPose) {
        Optional<Translation2d> aprilTagData = aprilTagVisionSubsystem.getRobotRelativeLocationOfAprilTag(targetCameraID, targetAprilTagID);
        akitLog.record("AprilTagData", aprilTagData.orElse(null));
        // This transform will always be at rotation 0, since in its own frame, the robot is always facing forward.

        if (aprilTagData.isEmpty()) {
            tagAcquisitionState = TagAcquisitionState.Lost;
            return;
        }

        // The aprilTagData has the robot-relative location of the AprilTag, but if we tried to drive into it we would crash
        // into the Reef/Coral station, since the robot has some width/depth. We will create a transform that includes an
        // X-offset (since X is the forward/backward direction) to account for this.
        Transform2d relativeGoalTransform = new Transform2d(
                aprilTagData.get().minus(alignmentPointOffset),
                new Rotation2d()
        );
        // Use WPI libraries to transform the relative goal into a field-oriented goal. That way, if we ever lose the tag,
        // we can still attempt to move to this target location
        targetLocationOnField = currentPose.transformBy(relativeGoalTransform).getTranslation();
    }

    public boolean recommendIsFinished() {
        return activity == Activity.Complete;
    }
}
