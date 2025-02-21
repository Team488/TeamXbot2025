package competition.subsystems.drive.commands;

import competition.subsystems.pose.EnumsToPose;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.pose.BasePoseSubsystem;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class SwerveBezierTrajectoryCommand extends SwerveSimpleTrajectoryCommand {

    private final CoprocessorCommunicationSubsystem coprocessor;

    // --- NEW CONSTANTS ---
    private final EnumsToPose enumsToPose = new EnumsToPose(); // not sure why yall made a class that reinits each time, this should be final.
    private static final int STEPS_PER_SEGMENT = 20;
    private static final double DEFAULT_ACCELERATION = 1.0;
    private static final double DEFAULT_METERS_PER_SECOND_VELOCITY = 2.0;
    private final List<Pose2d> reefPoses;
    private final AprilTagFieldLayout layout;

    @Inject
    public SwerveBezierTrajectoryCommand(BaseSwerveDriveSubsystem drive, BasePoseSubsystem pose, PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory, RobotAssertionManager assertionManager, CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.coprocessor = coprocessorCommunicationSubsystem;
        this.layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        this.reefPoses = new ArrayList<>();
        final List<Optional<Pose3d>> optionalPoses = new ArrayList<>();
        optionalPoses.add(this.layout.getTagPose(19));
        optionalPoses.add(this.layout.getTagPose(18));
        optionalPoses.add(this.layout.getTagPose(17));
        optionalPoses.add(this.layout.getTagPose(20));
        optionalPoses.add(this.layout.getTagPose(21));
        optionalPoses.add(this.layout.getTagPose(22));
        optionalPoses.add(this.layout.getTagPose(10));
        optionalPoses.add(this.layout.getTagPose(11));
        optionalPoses.add(this.layout.getTagPose(9));
        optionalPoses.add(this.layout.getTagPose(8));
        optionalPoses.add(this.layout.getTagPose(7));
        optionalPoses.add(this.layout.getTagPose(6));
        this.reefPoses.addAll(optionalPoses.stream().filter(Optional::isPresent)
                .map(m -> m.get().toPose2d())
                .toList());
    }

    @Override
    public void initialize() {
        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        XTablesClient client = this.coprocessor.tryGetXTablesClient();
        if (client != null) {
            XTableValues.BezierCurves curves = client.getBezierCurves("bezier_path");
            if (curves != null && !curves.getCurvesList().isEmpty()) {
                final XTableValues.TraversalOptions options = curves.hasOptions() ? curves.getOptions() : null;
                double acceleration = DEFAULT_ACCELERATION;
                double metersPerSecond = DEFAULT_METERS_PER_SECOND_VELOCITY;
                if (options != null) {
                    if (options.hasMetersPerSecond()) {
                        metersPerSecond = options.getMetersPerSecond();
                    }
                    if (options.hasAccelerationMetersPerSecond()) {
                        acceleration = options.getAccelerationMetersPerSecond();
                    }
                }
                this.logic.setGlobalKinematicValues(new SwervePointKinematics(acceleration,
                        0, 0, metersPerSecond));
                setSegmentedBezierCurve(curves, options);
            }
        }
        super.initialize();
    }

    public void setSegmentedBezierCurve(XTableValues.BezierCurves bezierCurves, XTableValues.TraversalOptions options) {
        this.logic.setKeyPoints(getSegmentedBezierCurveSwervePoints(bezierCurves, options));
    }

    public List<XbotSwervePoint> getSegmentedBezierCurveSwervePoints(XTableValues.BezierCurves bezierCurves, XTableValues.TraversalOptions options) {
        List<XbotSwervePoint> fullTrajectory = new ArrayList<>();

        // Get the current robot pose.
        Translation2d currentStartPoint = pose.getCurrentPose2d().getTranslation();
        final Rotation2d overallStartRotation = pose.getCurrentPose2d().getRotation();

        // Final rotation comes from options (or defaults to 0°).
        double finalRotationDegrees = (options != null && options.hasFinalRotationDegrees()) ? options.getFinalRotationDegrees() : 0;
        Rotation2d finalRotation = new Rotation2d(Units.degreesToRadians(finalRotationDegrees));

        // Use metersPerSecond from options if provided.
        double speed = (options != null && options.hasMetersPerSecond()) ? options.getMetersPerSecond() : DEFAULT_METERS_PER_SECOND_VELOCITY;

        // Total number of segments and steps.
        int totalSegments = bezierCurves.getCurvesList().size();
        int totalSteps = totalSegments * STEPS_PER_SEGMENT;
        int globalStep = 0;

        // Boolean to indicate if we want to use AprilTag facing.
        boolean faceAprilTag = (options != null && options.hasFaceNearestReefAprilTag() && options.getFaceNearestReefAprilTag());
        // When to start AprilTag lock (as fraction of total path)
        double startAprilTagThreshold = (options != null && options.hasStartFaceNearestReefAprilTagPathThresholdPercentage())
                ? options.getStartFaceNearestReefAprilTagPathThresholdPercentage() / 100.0
                : 0.0;
        // When to end AprilTag lock (as fraction of total path)
        double endAprilTagThreshold = (options != null && options.hasEndFaceNearestReefAprilTagPathThresholdPercentage())
                ? options.getEndFaceNearestReefAprilTagPathThresholdPercentage() / 100.0
                : 0.5; // Default to 50% if not provided

        // Final rotation blending speed factor (for after the AprilTag phase)
        double turnSpeedFactor = (options != null && options.hasFinalRotationTurnSpeedFactor()) ? options.getFinalRotationTurnSpeedFactor() : 1.0;

        // Should we instantly snap to the nearest AprilTag? (If false, we gradually turn.)
        boolean snapToNearestAprilTag = (options != null && options.hasSnapToNearestAprilTag()) ? options.getSnapToNearestAprilTag() : true;

        // If snapping is false, then use this many degrees per step as the max rotation delta.
        double aprilTagTurnSpeedPerStepDegrees = (options != null && options.hasAprilTagRotationDegreesTurnSpeedFactorPerStep())
                ? options.getAprilTagRotationDegreesTurnSpeedFactorPerStep()
                : 10.0; // default value

        // Which direction to face the tag (default FRONT for 2025 robot).
        XTableValues.RobotDirection tagDirection = (options != null && options.hasFaceNearestReefAprilTagDirection())
                ? options.getFaceNearestReefAprilTagDirection()
                : XTableValues.RobotDirection.FRONT;

        // ---------------------------
        // State variables for AprilTag mode:
        // ---------------------------
        // When not snapping, we maintain the current AprilTag rotation to increment gradually.
        Rotation2d currentAprilTagRotation = null;
        // When leaving AprilTag mode we capture the last AprilTag-facing rotation to blend from.
        Rotation2d aprilTagRotationAtEnd = null;

        // For non-AprilTag behavior, we use a default threshold (50% progress).
        double originalThreshold = 0.5;

        // Process each Bézier segment.
        for (XTableValues.BezierCurve segment : bezierCurves.getCurvesList()) {
            // Gather control points.
            List<Translation2d> segmentControlPoints = new ArrayList<>();
            for (XTableValues.ControlPoint cp : segment.getControlPointsList()) {
                segmentControlPoints.add(new Translation2d(cp.getX(), cp.getY()));
            }
            if (segmentControlPoints.isEmpty()) {
                continue;
            }
            // The last control point is assumed to be the segment’s endpoint.
            Translation2d segmentEndPoint = segmentControlPoints.get(segmentControlPoints.size() - 1);
            List<Translation2d> internalControlPoints = new ArrayList<>();
            if (segmentControlPoints.size() > 1) {
                internalControlPoints.addAll(segmentControlPoints.subList(0, segmentControlPoints.size() - 1));
            }

            // Build the full list of points for de Casteljau’s algorithm.
            List<Translation2d> allPoints = new ArrayList<>();
            allPoints.add(currentStartPoint);
            allPoints.addAll(internalControlPoints);
            allPoints.add(segmentEndPoint);

            // Process each step in the current segment.
            for (int i = 1; i <= STEPS_PER_SEGMENT; i++) {
                globalStep++;
                double lerpFraction = i / (double) STEPS_PER_SEGMENT;
                Translation2d pointTranslation = deCasteljauIterative(allPoints, lerpFraction);

                // Compute global progress (0 to 1) along the entire trajectory.
                double globalProgress = globalStep / (double) totalSteps;

                Rotation2d targetRotation;

                if (faceAprilTag && (globalProgress >= startAprilTagThreshold) && (globalProgress <= endAprilTagThreshold)) {
                    // --- APRIL TAG ACTIVE REGION ---
                    double desiredTagAngle = computeAprilTagAngle(pointTranslation, tagDirection);
                    Rotation2d desiredTagRotation = new Rotation2d(desiredTagAngle);

                    if (snapToNearestAprilTag) {
                        // Instantly snap to the computed AprilTag angle.
                        targetRotation = desiredTagRotation;
                        currentAprilTagRotation = desiredTagRotation; // store for potential blending later
                    } else {
                        // Gradually adjust toward the desired AprilTag rotation.
                        if (currentAprilTagRotation == null) {
                            currentAprilTagRotation = overallStartRotation;
                        }
                        // Limit the change per step.
                        double maxDelta = Units.degreesToRadians(aprilTagTurnSpeedPerStepDegrees);
                        targetRotation = incrementRotationTowards(currentAprilTagRotation, desiredTagRotation, maxDelta);
                        currentAprilTagRotation = targetRotation;
                    }
                } else if (faceAprilTag && (globalProgress > endAprilTagThreshold)) {
                    // --- BLENDING REGION (after AprilTag active) ---
                    if (aprilTagRotationAtEnd == null) {
                        // Capture the last AprilTag rotation (if available)
                        if (currentAprilTagRotation != null) {
                            aprilTagRotationAtEnd = currentAprilTagRotation;
                        } else {
                            double aprilTagAngle = computeAprilTagAngle(pointTranslation, tagDirection);
                            aprilTagRotationAtEnd = new Rotation2d(aprilTagAngle);
                        }
                    }
                    // Define an interpolation period that is shortened or lengthened by turnSpeedFactor.
                    double interpolationEnd = endAprilTagThreshold + (1 - endAprilTagThreshold) / turnSpeedFactor;
                    if (interpolationEnd > 1.0) {
                        interpolationEnd = 1.0;
                    }
                    if (globalProgress < interpolationEnd) {
                        double t = (globalProgress - endAprilTagThreshold) / (interpolationEnd - endAprilTagThreshold);
                        targetRotation = interpolateRotation(aprilTagRotationAtEnd, finalRotation, t);
                    } else {
                        targetRotation = finalRotation;
                    }
                } else {
                    // --- DEFAULT (NON-AprilTag) BEHAVIOR ---
                    // This applies when faceAprilTag is false or before the start threshold.
                    if (globalProgress <= originalThreshold) {
                        double t = globalProgress / originalThreshold;
                        targetRotation = interpolateRotation(overallStartRotation, finalRotation, t);
                    } else {
                        targetRotation = finalRotation;
                    }
                }

                fullTrajectory.add(new XbotSwervePoint(pointTranslation, targetRotation, speed));
            }

            // Update the starting point for the next segment.
            currentStartPoint = segmentEndPoint;
        }
        return fullTrajectory;
    }

    public List<XbotSwervePoint> getMultiSegmentedBezierCurveSwervePoints(List<XTableValues.BezierCurves> multiBezierCurves) {
        List<XbotSwervePoint> fullTrajectory = new ArrayList<>();
        for (XTableValues.BezierCurves curve : multiBezierCurves) {

            final XTableValues.TraversalOptions options = curve.hasOptions() ? curve.getOptions() : null;

            // Get the current robot pose.
            Translation2d currentStartPoint = pose.getCurrentPose2d().getTranslation();
            final Rotation2d overallStartRotation = pose.getCurrentPose2d().getRotation();

            // Final rotation comes from options (or defaults to 0°).
            double finalRotationDegrees = (options != null && options.hasFinalRotationDegrees()) ? options.getFinalRotationDegrees() : 0;
            Rotation2d finalRotation = new Rotation2d(Units.degreesToRadians(finalRotationDegrees));

            // Use metersPerSecond from options if provided.
            double speed = (options != null && options.hasMetersPerSecond()) ? options.getMetersPerSecond() : DEFAULT_METERS_PER_SECOND_VELOCITY;

            // Total number of segments and steps.
            int totalSegments = curve.getCurvesList().size();
            int totalSteps = totalSegments * STEPS_PER_SEGMENT;
            int globalStep = 0;

            // Boolean to indicate if we want to use AprilTag facing.
            boolean faceAprilTag = (options != null && options.hasFaceNearestReefAprilTag() && options.getFaceNearestReefAprilTag());
            // When to start AprilTag lock (as fraction of total path)
            double startAprilTagThreshold = (options != null && options.hasStartFaceNearestReefAprilTagPathThresholdPercentage())
                    ? options.getStartFaceNearestReefAprilTagPathThresholdPercentage() / 100.0
                    : 0.0;
            // When to end AprilTag lock (as fraction of total path)
            double endAprilTagThreshold = (options != null && options.hasEndFaceNearestReefAprilTagPathThresholdPercentage())
                    ? options.getEndFaceNearestReefAprilTagPathThresholdPercentage() / 100.0
                    : 0.5; // Default to 50% if not provided

            // Final rotation blending speed factor (for after the AprilTag phase)
            double turnSpeedFactor = (options != null && options.hasFinalRotationTurnSpeedFactor()) ? options.getFinalRotationTurnSpeedFactor() : 1.0;

            // Should we instantly snap to the nearest AprilTag? (If false, we gradually turn.)
            boolean snapToNearestAprilTag = (options != null && options.hasSnapToNearestAprilTag()) ? options.getSnapToNearestAprilTag() : true;

            // If snapping is false, then use this many degrees per step as the max rotation delta.
            double aprilTagTurnSpeedPerStepDegrees = (options != null && options.hasAprilTagRotationDegreesTurnSpeedFactorPerStep())
                    ? options.getAprilTagRotationDegreesTurnSpeedFactorPerStep()
                    : 10.0; // default value

            // Which direction to face the tag (default FRONT for 2025 robot).
            XTableValues.RobotDirection tagDirection = (options != null && options.hasFaceNearestReefAprilTagDirection())
                    ? options.getFaceNearestReefAprilTagDirection()
                    : XTableValues.RobotDirection.FRONT;

            // ---------------------------
            // State variables for AprilTag mode:
            // ---------------------------
            // When not snapping, we maintain the current AprilTag rotation to increment gradually.
            Rotation2d currentAprilTagRotation = null;
            // When leaving AprilTag mode we capture the last AprilTag-facing rotation to blend from.
            Rotation2d aprilTagRotationAtEnd = null;

            // For non-AprilTag behavior, we use a default threshold (50% progress).
            double originalThreshold = 0.5;

            // Process each Bézier segment.
            for (XTableValues.BezierCurve segment : curve.getCurvesList()) {
                // Gather control points.
                List<Translation2d> segmentControlPoints = new ArrayList<>();
                for (XTableValues.ControlPoint cp : segment.getControlPointsList()) {
                    segmentControlPoints.add(new Translation2d(cp.getX(), cp.getY()));
                }
                if (segmentControlPoints.isEmpty()) {
                    continue;
                }
                // The last control point is assumed to be the segment’s endpoint.
                Translation2d segmentEndPoint = segmentControlPoints.get(segmentControlPoints.size() - 1);
                List<Translation2d> internalControlPoints = new ArrayList<>();
                if (segmentControlPoints.size() > 1) {
                    internalControlPoints.addAll(segmentControlPoints.subList(0, segmentControlPoints.size() - 1));
                }

                // Build the full list of points for de Casteljau’s algorithm.
                List<Translation2d> allPoints = new ArrayList<>();
                allPoints.add(currentStartPoint);
                allPoints.addAll(internalControlPoints);
                allPoints.add(segmentEndPoint);

                // Process each step in the current segment.
                for (int i = 1; i <= STEPS_PER_SEGMENT; i++) {
                    globalStep++;
                    double lerpFraction = i / (double) STEPS_PER_SEGMENT;
                    Translation2d pointTranslation = deCasteljauIterative(allPoints, lerpFraction);

                    // Compute global progress (0 to 1) along the entire trajectory.
                    double globalProgress = globalStep / (double) totalSteps;

                    Rotation2d targetRotation;

                    if (faceAprilTag && (globalProgress >= startAprilTagThreshold) && (globalProgress <= endAprilTagThreshold)) {
                        // --- APRIL TAG ACTIVE REGION ---
                        double desiredTagAngle = computeAprilTagAngle(pointTranslation, tagDirection);
                        Rotation2d desiredTagRotation = new Rotation2d(desiredTagAngle);

                        if (snapToNearestAprilTag) {
                            // Instantly snap to the computed AprilTag angle.
                            targetRotation = desiredTagRotation;
                            currentAprilTagRotation = desiredTagRotation; // store for potential blending later
                        } else {
                            // Gradually adjust toward the desired AprilTag rotation.
                            if (currentAprilTagRotation == null) {
                                currentAprilTagRotation = overallStartRotation;
                            }
                            // Limit the change per step.
                            double maxDelta = Units.degreesToRadians(aprilTagTurnSpeedPerStepDegrees);
                            targetRotation = incrementRotationTowards(currentAprilTagRotation, desiredTagRotation, maxDelta);
                            currentAprilTagRotation = targetRotation;
                        }
                    } else if (faceAprilTag && (globalProgress > endAprilTagThreshold)) {
                        // --- BLENDING REGION (after AprilTag active) ---
                        if (aprilTagRotationAtEnd == null) {
                            // Capture the last AprilTag rotation (if available)
                            if (currentAprilTagRotation != null) {
                                aprilTagRotationAtEnd = currentAprilTagRotation;
                            } else {
                                double aprilTagAngle = computeAprilTagAngle(pointTranslation, tagDirection);
                                aprilTagRotationAtEnd = new Rotation2d(aprilTagAngle);
                            }
                        }
                        // Define an interpolation period that is shortened or lengthened by turnSpeedFactor.
                        double interpolationEnd = endAprilTagThreshold + (1 - endAprilTagThreshold) / turnSpeedFactor;
                        if (interpolationEnd > 1.0) {
                            interpolationEnd = 1.0;
                        }
                        if (globalProgress < interpolationEnd) {
                            double t = (globalProgress - endAprilTagThreshold) / (interpolationEnd - endAprilTagThreshold);
                            targetRotation = interpolateRotation(aprilTagRotationAtEnd, finalRotation, t);
                        } else {
                            targetRotation = finalRotation;
                        }
                    } else {
                        // --- DEFAULT (NON-AprilTag) BEHAVIOR ---
                        // This applies when faceAprilTag is false or before the start threshold.
                        if (globalProgress <= originalThreshold) {
                            double t = globalProgress / originalThreshold;
                            targetRotation = interpolateRotation(overallStartRotation, finalRotation, t);
                        } else {
                            targetRotation = finalRotation;
                        }
                    }

                    fullTrajectory.add(new XbotSwervePoint(pointTranslation, targetRotation, speed));
                }

                // Update the starting point for the next segment.
                currentStartPoint = segmentEndPoint;
            }
        }

        return fullTrajectory;
    }


    /**
     * Gradually increments the current rotation toward the target rotation,
     * but never by more than maxDelta (in radians) per step.
     */
    private Rotation2d incrementRotationTowards(Rotation2d current, Rotation2d target, double maxDelta) {
        double currentAngle = current.getRadians();
        double targetAngle = target.getRadians();
        // Calculate smallest angular difference, wrapped to [-PI, PI].
        double delta = ((targetAngle - currentAngle + Math.PI) % (2 * Math.PI)) - Math.PI;
        if (Math.abs(delta) > maxDelta) {
            delta = Math.signum(delta) * maxDelta;
        }
        return new Rotation2d(currentAngle + delta);
    }
// ************************************************************************
// HELPER METHODS
// ************************************************************************

    /**
     * Computes the rotation (in radians) the robot should have in order to face the nearest AprilTag.
     * The robot’s current position is given by 'robotPosition'. If 'direction' is BACK, then 180° is added.
     */
    private double computeAprilTagAngle(Translation2d robotPosition, XTableValues.RobotDirection direction) {
        // Assume pose2dHashMap exists and is populated.
        Pose2d nearestTag = null;
        double nearestDistance = Double.POSITIVE_INFINITY;
        for (Pose2d tagPose : reefPoses) {
            double distance = robotPosition.getDistance(tagPose.getTranslation());
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestTag = tagPose;
            }
        }
        // Fallback: if no tag is found, simply return the robot’s current heading.
        if (nearestTag == null) {
            return pose.getCurrentPose2d().getRotation().getRadians();
        }
        double dx = nearestTag.getTranslation().getX() - robotPosition.getX();
        double dy = nearestTag.getTranslation().getY() - robotPosition.getY();
        double angle = Math.atan2(dy, dx);
        if (direction == XTableValues.RobotDirection.BACK) {
            angle += Math.PI;
        }
        return angle;
    }


    /**
     * Linearly interpolates between two rotations.
     * This simple interpolation accounts for angle wrap‐around.
     */
    private Rotation2d interpolateRotation(Rotation2d start, Rotation2d end, double t) {
        double startAngle = start.getRadians();
        double endAngle = end.getRadians();
        // Wrap the difference to [-PI, PI] for smooth interpolation.
        double delta = ((endAngle - startAngle + Math.PI) % (2 * Math.PI)) - Math.PI;
        double interpolatedAngle = startAngle + t * delta;
        return new Rotation2d(interpolatedAngle);
    }


    /**
     * Iterative implementation of de Casteljau's algorithm to compute a point on a Bézier curve.
     *
     * @param points       The control points defining the curve.
     * @param lerpFraction The parametric value (0 to 1) along the curve.
     * @return The computed position as a Translation2d.
     */
    private Translation2d deCasteljauIterative(List<Translation2d> points, double lerpFraction) {
        int n = points.size();
        List<Translation2d> temp = new ArrayList<>(points);

        for (int level = 1; level < n; level++) {
            for (int i = 0; i < n - level; i++) {
                double x = (1 - lerpFraction) * temp.get(i).getX() + lerpFraction * temp.get(i + 1).getX();
                double y = (1 - lerpFraction) * temp.get(i).getY() + lerpFraction * temp.get(i + 1).getY();
                temp.set(i, new Translation2d(x, y));
            }
        }
        return temp.get(0);
    }

}