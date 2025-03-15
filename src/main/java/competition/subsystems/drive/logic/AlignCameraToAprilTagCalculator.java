package competition.subsystems.drive.logic;

import competition.electrical_contract.ElectricalContract;
import competition.operator_interface.OperatorInterface;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
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

    private static final Logger log = LoggerFactory.getLogger(AlignCameraToAprilTagCalculator.class);

    public enum TagAcquisitionState {
        NeverSeen,
        LockedOn,
        Lost
    }

    public enum Activity {
        Searching,
        ApproachWhileCentering,
        TerminalApproach,
        TerminalApproachWithoutVision,
        Shove,
        ShoveWithVision,
        Complete,
        GaveUp
    }

    public enum ApproachMode {
        Close,
        Far
    }

    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    final HeadingModule headingModule;
    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final AKitLogger akitLog;
    final ElectricalContract electricalContract;
    final ReefCoordinateGenerator reefCoordinateGenerator;
    final OperatorInterface oi;
    final AlignWithCreeperCalculator creeperCalculator;

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
    final DoubleProperty distanceFromInterstitialToAdvanceFast;
    final DoubleProperty distanceFromInterstitialToAdvanceSlow;
    final DoubleProperty approachSpeedFactor;
    final DoubleProperty distanceToStartShoving;
    final DoubleProperty shovePower;
    final DoubleProperty shoveDuration;
    final DoubleProperty maxTagAmbiguity;
    final DoubleProperty maxHorizontalErrorMeters;
    final DoubleProperty maxCreeperDistanceInMeters;

    final DoubleProperty closeInterstitialDistance;
    final DoubleProperty closeApproachSpeedFactor;
    final DoubleProperty closeInterstitialActivationRange;

    double lastKnownHorizontalErrorMeters = 999999;
    double shoveStartTime = 0;
    private Activity startingActivity = Activity.Searching;
    private boolean requireExcellentAlignment = true;

    Translation2d aprilTagPositionInGlobalFieldCoordinates;
    double aprilTagZRotationRadians;
    double idealFinalHeadingDegrees;
    Pose2d interstitialPoint;
    private ApproachMode approachMode;

    private boolean hasEverSeenAprilTag = false;
    private Translation2d coralStationPreShovePoint;

    // Creeper stuff
    private boolean useVisionCreeperAlignment = true;
    private boolean canUseVisionCreeperAlignment = false;
    private int creeperFailCount = 0;
    private final int creeperFailTolerance = 3;
    private double previousSidewaysPower = 0;

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
                                           PropertyFactory pf, OperatorInterface oi,
                                           AlignWithCreeperCalculator creeperCalculator) {
        this.aprilTagVisionSubsystem = vision;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.drive = drive;
        this.pose = pose;
        this.electricalContract = electricalContract;
        this.reefCoordinateGenerator = reefCoordinateGenerator;
        this.akitLog = new AKitLogger(prefix);
        this.oi = oi;
        this.creeperCalculator = creeperCalculator;

        pf.setPrefix(prefix);
        interstitialDistance = pf.createPersistentProperty("InterstitialDistance-m", 2.25);
        distanceFromInterstitialToAdvanceFast = pf.createPersistentProperty("DistanceFromInterstitialToAdvanceFast-m", 0.4);
        distanceFromInterstitialToAdvanceSlow = pf.createPersistentProperty("DistanceFromInterstitialToAdvanceSlow-m", 0.2);
        approachSpeedFactor = pf.createPersistentProperty("ApproachSpeedFactor", 0.65);
        distanceToStartShoving = pf.createPersistentProperty("DistanceToStartShoving-m", 0.0762); // 3 inches
        shovePower = pf.createPersistentProperty("ShovePower", 0.25);
        shoveDuration = pf.createPersistentProperty("ShoveDuration-s", 0.25);
        maxTagAmbiguity = pf.createPersistentProperty("MaxTagAmbiguity", 0.5);
        maxHorizontalErrorMeters = pf.createPersistentProperty("MaxHorizontalError-m", 0.0508); // 2 inches
        maxCreeperDistanceInMeters = pf.createPersistentProperty("MaxCreeperDistance-m", 0.1016); // 4 inches

        closeInterstitialDistance = pf.createPersistentProperty("CloseInterstitialDistance-m", 1.0);
        closeApproachSpeedFactor = pf.createPersistentProperty("CloseApproachSpeedFactor", 0.33);
        closeInterstitialActivationRange = pf.createPersistentProperty("CloseInterstitialActivationRange-m", 1.33);

        reset();
    }

    private void reset() {
        drive.getPositionalPid().reset();
        tagAcquisitionState = TagAcquisitionState.NeverSeen;
        activity = startingActivity;
    }

    public void configureAndReset(int targetAprilTagID, int targetCameraID, Distance offset,
                                  boolean isCameraBackwards) {
        configureAndReset(targetAprilTagID, targetCameraID, offset, isCameraBackwards, Activity.Searching, true);
    }

    public void configureAndReset(int targetAprilTagID, int targetCameraID, Distance offset,
                                  boolean isCameraBackwards, Activity startingActivity,
                                    boolean requireExcellentAlignment) {
        this.startingActivity = startingActivity;
        this.requireExcellentAlignment = requireExcellentAlignment;
        this.targetAprilTagID = targetAprilTagID;
        this.targetCameraID = targetCameraID;
        this.isCameraBackwards = isCameraBackwards;
        var currentPose = pose.getCurrentPose2d();

        reset();

        this.initialHeading = currentPose.getRotation().getDegrees();

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

        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        if (elementType == Landmarks.FieldElementType.REEF_FACE) {
            var farPoint = reefCoordinateGenerator.getPoseRelativeToReefFaceAndBranch(
                    alliance,
                    Landmarks.getReefFaceFromTagId(targetAprilTagID),
                    isLeft ? Landmarks.Branch.B : Landmarks.Branch.A,
                    Meters.of(interstitialDistance.get()),
                    Meters.zero()
            );
            var closePoint = reefCoordinateGenerator.getPoseRelativeToReefFaceAndBranch(
                    alliance,
                    Landmarks.getReefFaceFromTagId(targetAprilTagID),
                    isLeft ? Landmarks.Branch.B : Landmarks.Branch.A,
                    Meters.of(closeInterstitialDistance.get()),
                    Meters.zero()
            );

            double farDistance = farPoint.getTranslation().getDistance(currentPose.getTranslation());
            double closeDistance = closePoint.getTranslation().getDistance(currentPose.getTranslation());

            if (closeDistance < farDistance && closeDistance < closeInterstitialActivationRange.get()) {
                approachMode = ApproachMode.Close;
                interstitialPoint = closePoint;
            } else {
                approachMode = ApproachMode.Far;
                interstitialPoint = farPoint;
            }
        } else if (elementType == Landmarks.FieldElementType.CORAL_STATION) {
            // From the Tag ID, figure out which coral station this is
            var station = Landmarks.getCoralStationFromTagId(targetAprilTagID);
            // From the station, get its pose
            var coralStationPose = PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.getCoralStationSectionPose(station, Landmarks.CoralStationSection.MID));
            // From the pose, project a point in front of it
            var vectorToInterstitialPoint = new Translation2d(interstitialDistance.get(), coralStationPose.getRotation());
            interstitialPoint = new Pose2d(
                    coralStationPose.getTranslation().plus(vectorToInterstitialPoint),
                    coralStationPose.getRotation()
            );
        }

        if (useVisionCreeperAlignment) {
            // Consider try to initialize this multiple times in this future
            creeperCalculator.setCamera(targetCameraID);
            canUseVisionCreeperAlignment = creeperCalculator.initialize();
        }

        akitLog.record("InterstitialPoint", interstitialPoint);
    }


    public AlignCameraToAprilTagAdvice getXYPowersAlignToAprilTag(Pose2d currentPose) {
        // First, let's get any evergreen information we will need in almost all state machines.
        // Mostly, this is about where we should be pointing - and we generally point at the tag unless we are fairly close.
        Optional<AprilTagVisionIO.TargetObservation> targetObservation = aprilTagVisionSubsystem.getTargetObservation(targetCameraID, targetAprilTagID);
        boolean doWeSeeOurTargetTag = targetObservation.isPresent() && targetObservation.get().ambiguity() < maxTagAmbiguity.get();
        hasEverSeenAprilTag |= doWeSeeOurTargetTag;

        Translation2d currentTranslation = pose.getCurrentPose2d().getTranslation();
        double headingToPointAtAprilTag = Radians.of(
                currentTranslation.minus(aprilTagPositionInGlobalFieldCoordinates).getAngle().getRadians() + Math.PI
        ).plus(Radians.of(isCameraBackwards ? Math.PI : 0)).in(Degrees);

        double approachSpeedFactor = (approachMode == ApproachMode.Close)
                ? closeApproachSpeedFactor.get()
                : this.approachSpeedFactor.get();

        // Eventually we need to return these - they will likely be mutated by later steps.
        XYPair driveIntent = new XYPair(0, 0);
        double rotationIntent = 0;

        // To allow for quick drop-through, we will have the first step of the state machine be an "if" statement
        // so that if we see the tag we can jump right into the meat of the machine.
        if (activity == Activity.Searching) {
            if (doWeSeeOurTargetTag) {
                // We see the tag, we can begin approaching
                activity = Activity.ApproachWhileCentering;
            } else {
                return new AlignCameraToAprilTagAdvice(
                            driveIntent,
                            headingModule.calculateHeadingPower(headingToPointAtAprilTag),
                            tagAcquisitionState, activity);
            }
        }

        switch (activity) {
            case ApproachWhileCentering -> {
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
                driveIntent = new XYPair(normalizedVector.getX(), normalizedVector.getY()).scale(approachSpeedFactor);
                rotationIntent = headingModule.calculateHeadingPower(headingToPointAtAprilTag);

                // Finally, a check to see if we're quite close and should advance to the next state.

                var distanceThresholdToAdvance = (approachMode == ApproachMode.Close)
                        ? distanceFromInterstitialToAdvanceSlow.get()
                        : distanceFromInterstitialToAdvanceFast.get();

                if (currentTranslation.getDistance(interstitialPoint.getTranslation()) < distanceThresholdToAdvance) {
                    activity = Activity.TerminalApproach;

                    // Trying to approach coral station but found no tag? We'll just use pure pose
                    if (!hasEverSeenAprilTag && isCameraBackwards) {
                        activity = Activity.TerminalApproachWithoutVision;
                        coralStationPreShovePoint = getCoralStationPreShovePoint();
                        pose.setPreferOdometryToVision(false);
                    }
                }
            }
            case TerminalApproach -> {
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
                var powers = drive.getPowerToAchieveFieldPosition(currentTranslation, targetLocationOnField);

                // If we are going too fast, cap the speed.
                if (powers.getNorm() > approachSpeedFactor) {
                    powers = new Translation2d(approachSpeedFactor, powers.getAngle());
                }

                driveIntent = new XYPair(powers.getX(), powers.getY()).scale(approachSpeedFactor);
                rotationIntent = headingModule.calculateHeadingPower(idealFinalHeadingDegrees);

                // If we're quite close to the final point, advance to shoving into the reef or coral station.
                if (currentTranslation.getDistance(targetLocationOnField) < distanceToStartShoving.get()) {
                    // We need to make a decision. If our error is small enough, we should advance to shove.
                    // However, if our error is large, we should retreat and try again?

                    if (isLastKnownErrorWithinBounds() || !requireExcellentAlignment) {
                        shoveStartTime = XTimer.getFPGATimestamp();
                        activity = canUseVisionCreeperAlignment ? Activity.ShoveWithVision : Activity.Shove;
                    } else {
                        //activity = Activity.ApproachWhileCentering; // TODO: Retries ONLY if elevator not up high?
                    }
                }
            }
            case TerminalApproachWithoutVision -> {
                var noVisionPower = drive.getPowerToAchieveFieldPosition(currentTranslation, coralStationPreShovePoint);

                // If we are going too fast, cap the speed.
                if (noVisionPower.getNorm() > approachSpeedFactor) {
                    noVisionPower = new Translation2d(approachSpeedFactor, noVisionPower.getAngle());
                }

                driveIntent = new XYPair(noVisionPower.getX(), noVisionPower.getY()).scale(approachSpeedFactor);
                rotationIntent = headingModule.calculateHeadingPower(idealFinalHeadingDegrees);
                if (currentTranslation.getDistance(coralStationPreShovePoint) < distanceToStartShoving.get()) {
                    activity = Activity.Shove;
                    shoveStartTime = XTimer.getFPGATimestamp();
                }
            }
            case Shove -> {
                // We are so very close to our destination, but it's very hard to get perfect alignment -- the PID
                // will bring us close, but error in the dive might mean we are off by a few inches. We are also
                // so close to the april tag that it's no longer guaranteed to be in view. So, we will just try and
                // drive straight into the reef.

                // Additionally, we can also use some creeping logic to help us resolve any offsets.

                // We know the ideal angle the robot will be pointing at, so we can quickly construct a shove vector
                // in that direction.

                double shoveDirection = idealFinalHeadingDegrees;
                if (isCameraBackwards) {
                    shoveDirection += 180;
                }

                driveIntent = XYPair.fromPolar(shoveDirection, shovePower.get());
                rotationIntent = headingModule.calculateHeadingPower(idealFinalHeadingDegrees);

                // If we've been shoving for a while, we're done.
                if (XTimer.getFPGATimestamp() - shoveStartTime > shoveDuration.get()) {
                    activity = Activity.Complete;
                    oi.operatorGamepad.getRumbleManager().rumbleGamepad(1, .75);
                    oi.driverGamepad.getRumbleManager().rumbleGamepad(1, .75);
                }
            }
            case ShoveWithVision -> {
                // Use creeper when we are close enough to tag and horizontal error is small enough
                boolean useCreeper =
                        (currentTranslation.getDistance(targetLocationOnField) < maxCreeperDistanceInMeters.get())
                        && (lastKnownHorizontalErrorMeters < maxHorizontalErrorMeters.get());
                akitLog.record("UseCreeper", useCreeper); // Debugging

                // Generate standard forward shove power
                double shoveDirection = idealFinalHeadingDegrees;
                if (isCameraBackwards) {
                    shoveDirection += 180;
                }
                var forwardShove = XYPair.fromPolar(shoveDirection, shovePower.get());

                // Generate suggested creeping power
                var sidewaysShove = new XYPair(0, 0);
                if (useCreeper) {
                    var creeperSuggestion = creeperCalculator.getSuggestedSidewaysAlignmentPower();
                    akitLog.record("validCreeperSuggestion", creeperSuggestion.isSuggestionValid()); // Debugging

                    // We'll assume that we "skipped a heartbeat" and continue moving with previous power
                    // For situations where creeper subscriber isn't updating enough.
                    if (creeperSuggestion.isSuggestionValid()) {
                        previousSidewaysPower = creeperSuggestion.suggestedPower();
                        creeperFailCount = 0;
                        sidewaysShove = new XYPair(0, previousSidewaysPower);
                    } else if (creeperFailCount < creeperFailTolerance) {
                        creeperFailCount++;
                        sidewaysShove = new XYPair(0, previousSidewaysPower);
                    } else {
                        // Give up shoving sideways if fail more than 3 times && invalid suggestion
                        previousSidewaysPower = 0;
                        sidewaysShove = new XYPair(0, previousSidewaysPower);
                    }
                }

                driveIntent = forwardShove.add(sidewaysShove);
                rotationIntent = headingModule.calculateHeadingPower(idealFinalHeadingDegrees);

                // Very shaky ending condition...
                boolean creeperFinished = creeperCalculator.getIsConfidentlyCentered() || creeperFailCount >= 3;
                if (XTimer.getFPGATimestamp() - shoveStartTime > shoveDuration.get()
                        && (!useCreeper || creeperFinished)) {
                    activity = Activity.Complete;
                    oi.operatorGamepad.getRumbleManager().rumbleGamepad(1, .75);
                    oi.driverGamepad.getRumbleManager().rumbleGamepad(1, .75);
                }
            }
            case Complete -> {}
            case GaveUp -> {}
            default -> {} // We're done! We don't need to do anything.
        }

        akitLog.record("Activity", activity);
        akitLog.record("TagAcquisitionState", tagAcquisitionState);
        akitLog.record("ErrorIsWithinBounds", isLastKnownErrorWithinBounds());
        akitLog.record("LastKnownHorizontalErrorMeters", lastKnownHorizontalErrorMeters);

        return new AlignCameraToAprilTagAdvice(driveIntent, rotationIntent, tagAcquisitionState, activity);
    }

    private boolean isLastKnownErrorWithinBounds() {
        // We have to at least be in an Approach/Shove phase for this to be possibly true.
        // If we're in the searching phase, error isn't relevant, so we return false.
        return switch (activity) {
            case ApproachWhileCentering, TerminalApproach, Shove ->
                    Math.abs(lastKnownHorizontalErrorMeters) < maxHorizontalErrorMeters.get();
            default -> false;
        };
    }

    private void updateFinalTargetState(Pose2d currentPose) {
        Optional<Translation2d> aprilTagData = aprilTagVisionSubsystem.getRobotRelativeLocationOfAprilTag(targetCameraID, targetAprilTagID);

        if (aprilTagData.isPresent()) {
            lastKnownHorizontalErrorMeters = aprilTagData.get().getY();
        }
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
        akitLog.record("TargetLocationOnField", targetLocationOnField);
    }

    public boolean recommendIsFinished() {
        return activity == Activity.Complete || activity == Activity.GaveUp;
    }

    private Translation2d getCoralStationPreShovePoint() {
        var station = Landmarks.getCoralStationFromTagId(targetAprilTagID);
        var stationLocation = PoseSubsystem.convertBlueToRedIfNeeded(
                Landmarks.getCoralStationSectionPose(station, Landmarks.CoralStationSection.MID));

        var projectedDelta = new Translation2d(0.25, stationLocation.getRotation());
        return stationLocation.getTranslation().plus(projectedDelta);
    }
}
