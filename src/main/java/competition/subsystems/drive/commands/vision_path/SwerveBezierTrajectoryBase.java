package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleBezierCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.pose.BasePoseSubsystem;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class SwerveBezierTrajectoryBase extends SwerveSimpleBezierCommand {
    private final CoprocessorCommunicationSubsystem coprocessor;

    // --- NEW CONSTANTS ---
    private static final int STEPS_PER_SEGMENT = 15;
    private static final double DEFAULT_ACCELERATION = 1.0;
    private static final double DEFAULT_METERS_PER_SECOND_VELOCITY = 2.0;
    private final AprilTagFieldLayout layout;

    @Inject
    public SwerveBezierTrajectoryBase(BaseSwerveDriveSubsystem drive,
                                      BasePoseSubsystem pose, PropertyFactory pf,
                                      HeadingModule.HeadingModuleFactory headingModuleFactory,
                                      RobotAssertionManager assertionManager,
                                      CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.coprocessor = coprocessorCommunicationSubsystem;
        this.layout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    }

    @Override
    public void initialize() {
        this.logic.setPrioritizeRotationIfCloseToGoal(true);
        super.initialize();
    }

    public void setSegmentedBezierCurve(XTableValues.BezierCurves bezierCurves,
                                        XTableValues.TraversalOptions options) {
        this.logic.setKeyPoints(
                getSegmentedBezierCurveSwervePoints(bezierCurves, options));
    }


    public List<XbotSwervePoint> getSegmentedBezierCurveSwervePoints(
            XTableValues.BezierCurves bezierCurves,
            XTableValues.TraversalOptions options) {
        List<XbotSwervePoint> fullTrajectory = new ArrayList<>();

        // Get the current robot pose.
        Translation2d currentStartPoint = pose.getCurrentPose2d().getTranslation();
        XTableValues.BezierCurve lastCurve = bezierCurves.getCurvesList().get(bezierCurves.getCurvesCount() - 1);
        XTableValues.ControlPoint lastPoint = lastCurve.getControlPoints(lastCurve.getControlPointsCount() - 1);
        Translation2d endPoint = new Translation2d( lastPoint.getX(), lastPoint.getY());
        double distanceFromEnd = currentStartPoint.getDistance(endPoint);
        double halfDistanceFromEnd = distanceFromEnd / 1.4;


        final Rotation2d overallStartRotation =
                pose.getCurrentPose2d().getRotation();

        // Final rotation comes from options (or defaults to 0°).
        double finalRotationDegrees =
                (options != null && options.hasFinalRotationDegrees())
                        ? options.getFinalRotationDegrees()
                        : 0;
        Rotation2d finalRotation =
                new Rotation2d(Units.degreesToRadians(finalRotationDegrees));

        // Use metersPerSecond from options if provided.
        double acceleration =
                (options != null && options.hasAccelerationMetersPerSecond())
                        ? options.getAccelerationMetersPerSecond()
                        : DEFAULT_ACCELERATION;
        double speed = (options != null && options.hasMetersPerSecond())
                ? options.getMetersPerSecond()
                : DEFAULT_METERS_PER_SECOND_VELOCITY;
        this.logic.setVelocityMode(
                SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        this.logic.setGlobalKinematicValues(
                new SwervePointKinematics(acceleration, 0, 0, speed));
        // Total number of segments and steps.
        int totalSegments = bezierCurves.getCurvesList().size();
        int totalSteps = totalSegments * STEPS_PER_SEGMENT;
        int globalStep = 0;


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
            Translation2d segmentEndPoint =
                    segmentControlPoints.get(segmentControlPoints.size() - 1);
            List<Translation2d> internalControlPoints = new ArrayList<>();
            if (segmentControlPoints.size() > 1) {
                internalControlPoints.addAll(
                        segmentControlPoints.subList(0, segmentControlPoints.size() - 1));
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
                Translation2d pointTranslation =
                        deCasteljauIterative(allPoints, lerpFraction);
                Rotation2d targetRotation = overallStartRotation;
                if(pointTranslation.getDistance(endPoint) <= halfDistanceFromEnd) {
                    targetRotation = finalRotation;
                }

                fullTrajectory.add(
                        new XbotSwervePoint(pointTranslation, targetRotation, 1000));
            }

            // Update the starting point for the next segment.
            currentStartPoint = segmentEndPoint;
        }
        return fullTrajectory;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    /**
     * Iterative implementation of de Casteljau's algorithm to compute a point on
     * a Bézier curve.
     *
     * @param points       The control points defining the curve.
     * @param lerpFraction The parametric value (0 to 1) along the curve.
     * @return The computed position as a Translation2d.
     */
    private Translation2d deCasteljauIterative(
            List<Translation2d> points, double lerpFraction) {
        int n = points.size();
        List<Translation2d> temp = new ArrayList<>(points);

        for (int level = 1; level < n; level++) {
            for (int i = 0; i < n - level; i++) {
                double x = (1 - lerpFraction) * temp.get(i).getX()
                        + lerpFraction * temp.get(i + 1).getX();
                double y = (1 - lerpFraction) * temp.get(i).getY()
                        + lerpFraction * temp.get(i + 1).getY();
                temp.set(i, new Translation2d(x, y));
            }
        }
        return temp.get(0);
    }
}