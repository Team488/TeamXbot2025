package competition.subsystems.drive.commands;
import competition.subsystems.pose.EnumsToPose;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
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

public class SwerveBezierTrajectoryCommand extends SwerveSimpleTrajectoryCommand {

    private final CoprocessorCommunicationSubsystem coprocessor;

    // --- NEW CONSTANTS ---
    private final EnumsToPose enumsToPose = new EnumsToPose(); // not sure why yall made a class that reinits each time, this should be final.
    private static final int STEPS_PER_SEGMENT = 20;
    private static final double DEFAULT_ACCELERATION = 1.0;
    private static final double DEFAULT_METERS_PER_SECOND_VELOCITY = 2.0;

    @Inject
    public SwerveBezierTrajectoryCommand(BaseSwerveDriveSubsystem drive, BasePoseSubsystem pose, PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory, RobotAssertionManager assertionManager, CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.coprocessor = coprocessorCommunicationSubsystem;
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
        final int STEPS_PER_SEGMENT = 20; // Adjust as needed.
        int totalSteps = totalSegments * STEPS_PER_SEGMENT;
        int globalStep = 0;

        // AprilTag options.
        boolean faceAprilTag = (options != null && options.hasFaceNearestReefAprilTag() && options.getFaceNearestReefAprilTag());
        // The threshold percentage (e.g. 80.0) converted to fraction.
        double aprilTagThreshold = (options != null && options.hasFaceNearestReefAprilTagPathThresholdPercentage())
                ? options.getFaceNearestReefAprilTagPathThresholdPercentage() / 100.0
                : 0.0;
        // For non-AprilTag behavior, we use a default threshold (50%).
        double originalThreshold = 0.5;

        // How quickly to blend to the final rotation.
        double turnSpeedFactor = (options != null && options.hasFinalRotationTurnSpeedFactor()) ? options.getFinalRotationTurnSpeedFactor() : 1.0;

        // Which direction to face the tag (default FRONT for 2025 robot).
        XTableValues.RobotDirection tagDirection = (options != null && options.hasFaceNearestReefAprilTagDirection())
                ? options.getFaceNearestReefAprilTagDirection()
                : XTableValues.RobotDirection.FRONT;

        // This will capture the april tag–facing rotation at the threshold.
        Rotation2d aprilTagRotationAtThreshold = null;

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
                if (faceAprilTag && aprilTagThreshold > 0) {
                    // --- APRIL TAG MODE ---
                    if (globalProgress <= aprilTagThreshold) {
                        // For the early portion, face the nearest AprilTag.
                        double aprilTagAngle = computeAprilTagAngle(pointTranslation, tagDirection);
                        targetRotation = new Rotation2d(aprilTagAngle);

                        // Capture the rotation at the threshold (using the step that is closest to the threshold).
                        if (Math.abs(globalProgress - aprilTagThreshold) < (1.0 / totalSteps) || Math.abs(globalProgress - aprilTagThreshold) < 1e-3) {
                            aprilTagRotationAtThreshold = targetRotation;
                        }
                    } else {
                        // --- BLENDING MODE ---
                        // If we haven’t yet stored the rotation at threshold, do so now.
                        if (aprilTagRotationAtThreshold == null) {
                            double aprilTagAngle = computeAprilTagAngle(pointTranslation, tagDirection);
                            aprilTagRotationAtThreshold = new Rotation2d(aprilTagAngle);
                        }
                        // Determine how quickly to blend based on turnSpeedFactor.
                        double interpolationEnd = aprilTagThreshold + (1 - aprilTagThreshold) / turnSpeedFactor;
                        if (interpolationEnd > 1.0) {
                            interpolationEnd = 1.0;
                        }
                        if (globalProgress < interpolationEnd) {
                            double t = (globalProgress - aprilTagThreshold) / (interpolationEnd - aprilTagThreshold);
                            targetRotation = interpolateRotation(aprilTagRotationAtThreshold, finalRotation, t);
                        } else {
                            targetRotation = finalRotation;
                        }
                    }
                } else {
                    // If not using AprilTag rotation options, we interpolate from the overall start rotation to
                    // final rotation until reaching the default threshold (50% progress).
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
        logic.setKeyPoints(fullTrajectory);
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
        for (Pose2d tagPose : enumsToPose.getPose2dHashMap().values()) {
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
     * Note: This simple interpolation assumes the angular difference is small.
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