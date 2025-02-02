package competition.subsystems.oracle;

import edu.wpi.first.math.geometry.Translation2d;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class ReefCollisionCircle {
    private final Translation2d center;
    private final double radius;
    private final boolean isValid;

    private static Logger log = LogManager.getLogger(ReefCollisionCircle.class);

    public ReefCollisionCircle(Translation2d center, double radius) {
        if (center == null || radius <= 0) {
            log.warn("Invalid center or radius: center={}, radius={}", center, radius);
            this.center = new Translation2d(0, 0);
            this.radius = 0;
            this.isValid = false;
        } else {
            this.center = center;
            this.radius = radius;
            this.isValid = true;
        }
    }

    public boolean doesLineIntersect(Translation2d point1, Translation2d point2) {
        if (!isValid || point1 == null || point2 == null || point1.equals(point2)) {
            return false;
        }

        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        double fx = point1.getX() - center.getX();
        double fy = point1.getY() - center.getY();

        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = (fx * fx + fy * fy) - (radius * radius);

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return false;
        } else {
            double sqrtDiscriminant = Math.sqrt(discriminant);
            double t1 = (-b - sqrtDiscriminant) / (2 * a);
            double t2 = (-b + sqrtDiscriminant) / (2 * a);

            return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
        }
    }
}