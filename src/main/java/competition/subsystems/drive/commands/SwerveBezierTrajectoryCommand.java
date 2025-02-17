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
            if (curves != null && curves.getPathFound()) {
                // Use the new segmented integration method.
                this.logic.setGlobalKinematicValues(new SwervePointKinematics(curves.getAccelerationMetersPerSecond(), 0, 0, curves.getMetersPerSecond()));
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
     * - A constant speed (CONSTANT_SPEED)
     * - A constant final rotation
     * </p>
     */
    public void setSegmentedBezierCurve(XTableValues.BezierCurves curvesProto, double SPEED_MPS) {
        List<XbotSwervePoint> fullTrajectory = new ArrayList<>();
        // Start at the robot's current position.
        Translation2d currentStartPoint = pose.getCurrentPose2d().getTranslation();
        // Use the rotation from the configured endPoint if available; otherwise, use the default.
        Rotation2d finalRotation = new Rotation2d(Units.degreesToRadians(curvesProto.getFinalRotationDegrees()));

        // Iterate over each curve segment from the proto.
        for (XTableValues.BezierCurve segment : curvesProto.getCurvesList()) {
            // Convert proto control points into WPILib's Translation2d.
            List<Translation2d> segmentControlPoints = new ArrayList<>();
            for (XTableValues.ControlPoint cp : segment.getControlPointsList()) {
                segmentControlPoints.add(new Translation2d(cp.getX(), cp.getY()));
            }

            // If no control points, skip this segment.
            if (segmentControlPoints.isEmpty()) {
                continue;
            }

            // Assume the last control point is the segment's end point.
            Translation2d segmentEndPoint = segmentControlPoints.get(segmentControlPoints.size() - 1);
            // Any preceding points are treated as internal control points.
            List<Translation2d> internalControlPoints = new ArrayList<>();
            if (segmentControlPoints.size() > 1) {
                internalControlPoints.addAll(segmentControlPoints.subList(0, segmentControlPoints.size() - 1));
            }

            // Build the full list of points for de Casteljau:
            // [start, internal control points..., end]
            List<Translation2d> allPoints = new ArrayList<>();
            allPoints.add(currentStartPoint);
            allPoints.addAll(internalControlPoints);
            allPoints.add(segmentEndPoint);

            // Use a fixed number of steps per segment.
            for (int i = 1; i <= STEPS_PER_SEGMENT; i++) {
                double lerpFraction = i / (double) STEPS_PER_SEGMENT;
                Translation2d pointTranslation = deCasteljauIterative(allPoints, lerpFraction);
                // Instead of computing a tangent rotation, use the constant final rotation.
                fullTrajectory.add(new XbotSwervePoint(pointTranslation, finalRotation, SPEED_MPS));
            }

            // For the next segment, the start point is the current segment's endpoint.
            currentStartPoint = segmentEndPoint;
        }

        // Set the full, concatenated trajectory into your drive logic.
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

    public void setBezierCurve(List<Translation2d> controlPoints, Pose2d endPoint, int steps, double SPEED_MPS) {
        List<XbotSwervePoint> bezierPoints = new ArrayList<>();
        List<Translation2d> allPoints = new ArrayList<>();
        allPoints.add(pose.getCurrentPose2d().getTranslation());
        allPoints.addAll(controlPoints);
        allPoints.add(endPoint.getTranslation());

        // Instead of interpolating rotation, use the final rotation consistently.
        Rotation2d finalRotation = endPoint.getRotation();
        for (int i = 1; i <= steps; i++) {
            double lerpFraction = i / (double) steps;
            XbotSwervePoint point = new XbotSwervePoint(deCasteljauIterative(allPoints, lerpFraction), finalRotation, SPEED_MPS);
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
