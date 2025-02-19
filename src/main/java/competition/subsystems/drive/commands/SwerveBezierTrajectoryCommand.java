package competition.subsystems.drive.commands;


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

    // For the single-curve configuration (if needed)
    List<Translation2d> controlPoints;
    Pose2d endPoint;
    int steps;

    private CoprocessorCommunicationSubsystem coprocessor;

    // --- NEW CONSTANTS ---
    private static final int STEPS_PER_SEGMENT = 20;

    @Inject
    public SwerveBezierTrajectoryCommand(BaseSwerveDriveSubsystem drive,
                                         BasePoseSubsystem pose, PropertyFactory pf,
                                         HeadingModule.HeadingModuleFactory headingModuleFactory,
                                         RobotAssertionManager assertionManager,
                                         CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.coprocessor = coprocessorCommunicationSubsystem;
    }

    @Override
    public void initialize() {
        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        XTablesClient client = this.coprocessor.tryGetXTablesClient();
        if (client != null) {

            XTableValues.BezierCurves curves = client.getBezierCurves("bezier_path");
            if (curves != null && curves.getPathFound()) {
                // Use the new segmented integration method.
                this.logic.setGlobalKinematicValues(new SwervePointKinematics(curves.getAccelerationMetersPerSecond(),
                        0, 0, curves.getMetersPerSecond()));
                setSegmentedBezierCurve(curves, curves.getMetersPerSecond());
            }
        }
        super.initialize();

    }


    /**
     * Integrates multiple Bézier curve segments from the XTableValues into a single trajectory.
     * <p>
     * This version ignores timeToTraverse, metersPerSecondSpeed, and tangent rotation. It uses:
     * - A constant number of steps per segment (STEPS_PER_SEGMENT)
     * - A constant speedMps (CONSTANT_SPEED)
     * - A constant final rotation
     * </p>
     */
    public void setSegmentedBezierCurve(XTableValues.BezierCurves curvesProto, double speedMps) {
        List<XbotSwervePoint> fullTrajectory = new ArrayList<>();

        // Start at the robot's current position and rotation.
        Translation2d currentStartPoint = pose.getCurrentPose2d().getTranslation();
        Rotation2d overallStartRotation = pose.getCurrentPose2d().getRotation();

        // Compute the final rotation from the proto (converted from degrees to radians).
        Rotation2d finalRotation = new Rotation2d(Units.degreesToRadians(curvesProto.getFinalRotationDegrees()));

        // Total number of segments from the proto.
        int totalSegments = curvesProto.getCurvesList().size();

        // This variable holds the rotation at the start of the current segment.
        Rotation2d currentRotation = overallStartRotation;

        // Iterate over each curve segment.
        int segmentIndex = 0;
        for (XTableValues.BezierCurve segment : curvesProto.getCurvesList()) {
            segmentIndex++;

            // Process control points for the segment.
            List<Translation2d> segmentControlPoints = new ArrayList<>();
            for (XTableValues.ControlPoint cp : segment.getControlPointsList()) {
                segmentControlPoints.add(new Translation2d(cp.getX(), cp.getY()));
            }

            if (segmentControlPoints.isEmpty()) continue;

            // Assume the last control point is the segment's end point.
            Translation2d segmentEndPoint = segmentControlPoints.get(segmentControlPoints.size() - 1);

            // Internal control points
            List<Translation2d> internalControlPoints = new ArrayList<>();
            if (segmentControlPoints.size() > 1) {
                internalControlPoints.addAll(segmentControlPoints.subList(0, segmentControlPoints.size() - 1));
            }

            // Build the full list of points for de Casteljau's algorithm
            List<Translation2d> allPoints = new ArrayList<>();
            allPoints.add(currentStartPoint);
            allPoints.addAll(internalControlPoints);
            allPoints.add(segmentEndPoint);

            // Compute segmentFraction based on total progress
            double segmentFraction = (double) segmentIndex / totalSegments;

            // Compute the target rotation based on whether we are in the first or second half
            double targetAngle;
            if (segmentFraction <= 0.5) {
                // First half: interpolate towards finalRotation
                double progress = segmentFraction / 0.5; // Normalize 0 to 1
                targetAngle = overallStartRotation.getRadians() + progress * (finalRotation.getRadians() - overallStartRotation.getRadians());
            } else {
                // Second half: maintain finalRotation
                targetAngle = finalRotation.getRadians();
            }

            Rotation2d segmentTargetRotation = new Rotation2d(targetAngle);

            // Interpolate the rotation gradually from currentRotation to segmentTargetRotation.
            for (int i = 1; i <= STEPS_PER_SEGMENT; i++) {
                double lerpFraction = i / (double) STEPS_PER_SEGMENT;
                Translation2d pointTranslation = deCasteljauIterative(allPoints, lerpFraction);
                double interpolatedAngle = currentRotation.getRadians() + lerpFraction * (segmentTargetRotation.getRadians() - currentRotation.getRadians());
                Rotation2d pointRotation = new Rotation2d(interpolatedAngle);
                fullTrajectory.add(new XbotSwervePoint(pointTranslation, pointRotation, speedMps));
            }

            // Update currentRotation for next segment.
            currentRotation = segmentTargetRotation;

            // Update the start point for the next segment.
            currentStartPoint = segmentEndPoint;
        }

        logic.setKeyPoints(fullTrajectory);
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

    // Retained methods for backward compatibility if you need single-curve support.

    public void setBezierConfiguration(List<Translation2d> controlPoints, Pose2d endPoint, int steps) {
        this.controlPoints = controlPoints;
        this.steps = steps;
        this.endPoint = endPoint;
    }

    public void setBezierCurve(List<Translation2d> controlPoints, Pose2d endPoint, int steps, double speedMps) {
        List<XbotSwervePoint> bezierPoints = new ArrayList<>();
        List<Translation2d> allPoints = new ArrayList<>();
        allPoints.add(pose.getCurrentPose2d().getTranslation());
        allPoints.addAll(controlPoints);
        allPoints.add(endPoint.getTranslation());

        // Instead of interpolating rotation, use the final rotation consistently.
        Rotation2d finalRotation = endPoint.getRotation();
        for (int i = 1; i <= steps; i++) {
            double lerpFraction = i / (double) steps;
            XbotSwervePoint point = new XbotSwervePoint(deCasteljauIterative(allPoints, lerpFraction), finalRotation,
                    speedMps);
            bezierPoints.add(point);
        }

        logic.setKeyPoints(bezierPoints);
    }

    // Original recursive de Casteljau method (not used in the segmented integration)
    private Translation2d deCasteljau(List<Translation2d> points, double lerpFraction) {
        if (points.size() == 1) {
            return points.get(0);
        }
        List<Translation2d> newPoints = new ArrayList<>();
        for (int i = 0; i < points.size() - 1; i++) {
            Translation2d p1 = points.get(i);
            Translation2d p2 = points.get(i + 1);
            double x = (1 - lerpFraction) * p1.getX() + lerpFraction * p2.getX();
            double y = (1 - lerpFraction) * p1.getY() + lerpFraction * p2.getY();
            newPoints.add(new Translation2d(x, y));
        }
        return deCasteljau(newPoints, lerpFraction);
    }
}