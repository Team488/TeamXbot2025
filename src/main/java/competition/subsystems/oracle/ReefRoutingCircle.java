package competition.subsystems.oracle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.MockPowerDistributionPanel;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import xbot.common.trajectory.XbotSwervePoint;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class ReefRoutingCircle {
    private final Translation2d center;
    private final double radius;
    private final ReefCollisionCircle collisionCircle;
    private final double innerCollisionCircleRadius;

    private static Logger log = LogManager.getLogger(ReefRoutingCircle.class);

    public ReefRoutingCircle(Translation2d center, double routingRadius) {
        this(center, routingRadius, routingRadius * 0.85);
    }

    public ReefRoutingCircle(Translation2d center, double routingRadius, double collisionRadius) {
        this.center = center;
        this.radius = routingRadius;
        this.innerCollisionCircleRadius = collisionRadius;
        this.collisionCircle = new ReefCollisionCircle(center, collisionRadius);
    }

    public double getInnerCollisionCircleRadius() {
        return innerCollisionCircleRadius;
    }

    public Translation2d getCenter() {
        return center;
    }

    public double getRadius() {
        return radius;
    }

    public List<XbotSwervePoint> generateSwervePoints(Pose2d startingPose, Pose2d endingPose) {
        List<XbotSwervePoint> swervePoints = new ArrayList<>();
        Translation2d start = startingPose.getTranslation();
        Translation2d end = endingPose.getTranslation();

        if (!collisionCircle.doesLineIntersect(start, end)) {
            // No reason to add starting point for trajectories.
            //swervePoints.add(new XbotSwervePoint(startingPose, 0));
            swervePoints.add(new XbotSwervePoint(endingPose, 0));
            return swervePoints;
        }

        // No reason to add starting point for trajectories.
        //swervePoints.add(new XbotSwervePoint(startingPose, 0));

        Translation2d tangentPoint = findClosestTangentPoint(start, end);
        swervePoints.add(new XbotSwervePoint(new Pose2d(tangentPoint, endingPose.getRotation()), 0));

        int escape = 0;
        while (collisionCircle.doesLineIntersect(tangentPoint, end)) {
            escape++;
            tangentPoint = moveAlongCircumference(tangentPoint, end, 0.25);
            swervePoints.add(new XbotSwervePoint(new Pose2d(tangentPoint, endingPose.getRotation()), 0));
            if (escape > 100) {
                log.warn("Infinite loop detected in generateSwervePoints, breaking out!");
                break;
            }
        }

        swervePoints.add(new XbotSwervePoint(endingPose, 10));
        return swervePoints;
    }

    private Translation2d findClosestTangentPoint(Translation2d point, Translation2d endPoint) {
        double cx = center.getX();
        double cy = center.getY();
        double r = radius;
        double px = point.getX();
        double py = point.getY();

        // Distance from circle center to external point
        double dx = px - cx;
        double dy = py - cy;
        double dSq = dx * dx + dy * dy;
        double d = Math.sqrt(dSq);

        // Check for cases like already on the circle or inside the circle
        if ((d < 1e-12) || (d<r) || (Math.abs(d - r) < 1e-12)) {
            return point;
        }

        // Otherwise, there are two tangents.
        // 1. Calculate a = r^2 / d^2
        double a = (r * r) / (dSq);

        // 2. Point M = C + a*(P - C)
        double mx = cx + a * dx;
        double my = cy + a * dy;

        // 3. Vector w = ( - (y1 - y0), x1 - x0 ) = perpendicular to CP
        //    In our notation, w = ( -dy, dx )
        double wx = -dy;
        double wy = dx;

        // 4. Distance from M to each tangent point = b = (r/d) * sqrt(d^2 - r^2)
        double h = Math.sqrt(dSq - r * r);
        double b = (r / d) * h;

        // Length of w is d; normalize and scale by b
        double wxScaled = wx * (b / d);
        double wyScaled = wy * (b / d);

        // 5. The two tangent points T1 and T2
        double t1x = mx + wxScaled;
        double t1y = my + wyScaled;
        double t2x = mx - wxScaled;
        double t2y = my - wyScaled;

        Translation2d tangentPoint1 = new Translation2d(t1x, t1y);
        Translation2d tangentPoint2 = new Translation2d(t2x, t2y);

        // Choose the tangent point closest to the end point
        double distanceToEnd1 = tangentPoint1.getDistance(endPoint);
        double distanceToEnd2 = tangentPoint2.getDistance(endPoint);

        return distanceToEnd1 < distanceToEnd2 ? tangentPoint1 : tangentPoint2;
    }

    private Translation2d moveAlongCircumference(Translation2d currentPoint, Translation2d targetPoint, double distance) {
        double angleToTarget = Math.atan2(targetPoint.getY() - center.getY(), targetPoint.getX() - center.getX());
        double angleCurrent = Math.atan2(currentPoint.getY() - center.getY(), currentPoint.getX() - center.getX());
        double angleStep = distance / radius;

        double newAngle = angleCurrent + (angleToTarget > angleCurrent ? angleStep : -angleStep);
        return new Translation2d(center.getX() + radius * Math.cos(newAngle), center.getY() + radius * Math.sin(newAngle));
    }

    private Trajectory visualizeCircleAsTrajectory(Translation2d center, double radius, int numberOfSteps) {
        var wpiStates = new ArrayList<Trajectory.State>();
        double angleStep = 2 * Math.PI / numberOfSteps;

        for (int i = 0; i < numberOfSteps; i++) {
            double angle = i * angleStep;
            double x = center.getX() + radius * Math.cos(angle);
            double y = center.getY() + radius * Math.sin(angle);
            Pose2d pose = new Pose2d(x, y, new Rotation2d(angle));
            Trajectory.State state = new Trajectory.State();
            state.poseMeters = pose;
            wpiStates.add(state);
        }

        return new Trajectory(wpiStates);
    }

    public Trajectory visualizeInnerCollisionCircleAsTrajectory() {
        return visualizeCircleAsTrajectory(center, innerCollisionCircleRadius, 20);
    }

    public Trajectory visualizeOuterRoutingCircleAsTrajectory() {
        return visualizeCircleAsTrajectory(center, radius, 20);
    }
}