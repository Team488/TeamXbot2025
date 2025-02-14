package competition.subsystems.drive.commands;

import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

/**
 * A "not-so-simple" SwerveSimpleTrajectoryCommand that does a Bézier curve!~~
 * NOTE: Currently, rotation is neglected...
 * IMPORTANT: This thing may get *expensive* the more control points you got
 */
public class SwerveBezierTrajectoryCommand extends SwerveSimpleTrajectoryCommand {

    List<Translation2d> controlPoints;
    Pose2d endPoint;
    int steps;
    private CoprocessorCommunicationSubsystem coprocessor;

    @Inject
    public SwerveBezierTrajectoryCommand(BaseSwerveDriveSubsystem drive, BasePoseSubsystem pose, PropertyFactory pf,
                                         HeadingModule.HeadingModuleFactory headingModuleFactory, RobotAssertionManager assertionManager, CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.coprocessor = coprocessorCommunicationSubsystem;
    }

    @Override
    public void initialize() {
        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        this.logic.setGlobalKinematicValues(new SwervePointKinematics(0.5, 0, 0, 2));
        XTablesClient client = this.coprocessor.tryGetXTablesClient();
        if (client != null) {
            XTableValues.BezierCurve curve = client.getBezierCurve("bezier_path");
            List<XTableValues.ControlPoint> controlPoints = curve.getControlPointsList();
            XTableValues.ControlPoint lastPoint = controlPoints.get(controlPoints.size() - 1);
            Pose2d lastPose = new Pose2d(lastPoint.getX(), lastPoint.getY(), new Rotation2d(0));
            if (controlPoints.size() > 1) {
                setBezierCurve(controlPoints.stream().map(m -> new Translation2d(m.getX(), m.getY())).toList(), lastPose, 15);
            }
        }


        super.initialize();
    }

    /**
     * Set the configuration of the Bézier curve, we can't generate our curve immediately as our pose will change
     *
     * @param controlPoints of the Bézier curve
     * @param endPoint      of our command
     * @param steps         to split our Bézier curve into
     */
    public void setBezierConfiguration(List<Translation2d> controlPoints, Pose2d endPoint, int steps) {
        this.controlPoints = controlPoints;
        this.steps = steps;
        this.endPoint = endPoint;
    }

    public void setBezierCurve(List<Translation2d> controlPoints, Pose2d endPoint, int steps) {
        List<XbotSwervePoint> bezierPoints = new ArrayList<>();
        List<Translation2d> allPoints = new ArrayList<>();
        // Start from the current robot position
        allPoints.add(pose.getCurrentPose2d().getTranslation());
        allPoints.addAll(controlPoints);
        allPoints.add(endPoint.getTranslation());

        // Generate points along the Bézier curve
        for (int i = 1; i <= steps; i++) {
            double t = i / (double) steps;
            Translation2d pos = deCasteljauIterative(allPoints, t);
            // Compute the tangent (and corresponding heading) at this point
            Rotation2d tangentRotation = computeTangentRotation(allPoints, t);
            // Create the swerve point using the position and tangent heading
            XbotSwervePoint point = new XbotSwervePoint(pos, tangentRotation, 10);
            bezierPoints.add(point);
        }

        logic.setKeyPoints(bezierPoints);
    }

    /**
     * Computes the tangent rotation at a given parametric value (t) along the Bézier curve.
     * It uses a small offset (epsilon) to compute a finite-difference derivative.
     *
     * @param points The list of points that define the Bézier curve (start, control, end).
     * @param t      The parametric value (from 0 to 1) at which to compute the tangent.
     * @return       A Rotation2d representing the heading along the tangent.
     */
    private Rotation2d computeTangentRotation(List<Translation2d> points, double t) {
        double epsilon = 1e-3; // small offset for finite differences
        double t1 = t - epsilon;
        double t2 = t + epsilon;

        // Clamp the values to the valid range [0, 1]
        if (t1 < 0) {
            t1 = t;
            t2 = t + epsilon;
        } else if (t2 > 1) {
            t1 = t - epsilon;
            t2 = t;
        }

        Translation2d p1 = deCasteljauIterative(points, t1);
        Translation2d p2 = deCasteljauIterative(points, t2);

        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();
        double angle = Math.atan2(dy, dx);

        return new Rotation2d(angle);
    }


    // ChatGPT said that this is better
    private Translation2d deCasteljauIterative(List<Translation2d> points, double lerpFraction) {
        int n = points.size();
        List<Translation2d> temp = new ArrayList<>(points);

        // Compute the position using de Casteljau's algorithm
        for (int level = 1; level < n; level++) {
            for (int i = 0; i < n - level; i++) {
                double x = (1 - lerpFraction) * temp.get(i).getX() + lerpFraction * temp.get(i + 1).getX();
                double y = (1 - lerpFraction) * temp.get(i).getY() + lerpFraction * temp.get(i + 1).getY();
                temp.set(i, new Translation2d(x, y)); // Update in place
            }
        }

        return temp.get(0);
    }

    /**
     * Generated by ChatGPT, this tells us where we SHOULD be at
     *
     * @param points       include: start, control points, end
     * @param lerpFraction is our completion percentage
     * @return our position during lerpFraction (progress of operation completion)
     */
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
