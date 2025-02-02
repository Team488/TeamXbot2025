package competition.subsystems.oracle;

import competition.BaseCompetitionTest;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.Test;
import static org.junit.Assert.*;

public class ReefCollisionCircleTest extends BaseCompetitionTest {

    @Test
    public void testNoIntersection() {
        ReefCollisionCircle circle = new ReefCollisionCircle(new Translation2d(0, 0), 5);
        Translation2d point1 = new Translation2d(-10, 0);
        Translation2d point2 = new Translation2d(-6, 0);
        assertFalse(circle.doesLineIntersect(point1, point2));
    }

    @Test
    public void testIntersection() {
        ReefCollisionCircle circle = new ReefCollisionCircle(new Translation2d(0, 0), 5);
        Translation2d point1 = new Translation2d(-10, 0);
        Translation2d point2 = new Translation2d(10, 0);
        assertTrue(circle.doesLineIntersect(point1, point2));
    }

    @Test
    public void testTangentIntersection() {
        ReefCollisionCircle circle = new ReefCollisionCircle(new Translation2d(0, 0), 5);
        Translation2d point1 = new Translation2d(5, -5);
        Translation2d point2 = new Translation2d(5, 5);
        assertTrue(circle.doesLineIntersect(point1, point2));
    }

    @Test
    public void testPointOnCircle() {
        ReefCollisionCircle circle = new ReefCollisionCircle(new Translation2d(0, 0), 5);
        Translation2d point1 = new Translation2d(5, 0);
        Translation2d point2 = new Translation2d(5, 0);
        assertFalse(circle.doesLineIntersect(point1, point2));
    }

    @Test
    public void testInvalidCircle() {
        ReefCollisionCircle circle = new ReefCollisionCircle(null, -1);
        Translation2d point1 = new Translation2d(0, 0);
        Translation2d point2 = new Translation2d(1, 1);
        assertFalse(circle.doesLineIntersect(point1, point2));
    }
}