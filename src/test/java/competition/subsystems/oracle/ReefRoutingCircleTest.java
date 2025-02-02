package competition.subsystems.oracle;

import competition.BaseCompetitionTest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import org.junit.Test;
import xbot.common.trajectory.XbotSwervePoint;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ReefRoutingCircleTest extends BaseCompetitionTest {

    @Test
    public void testGenerateSwervePointsNoIntersection() {
        Translation2d center = new Translation2d(0, 0);
        double radius = 2.0;
        ReefRoutingCircle routingCircle = new ReefRoutingCircle(center, radius);

        Pose2d startPose = new Pose2d(new Translation2d(-4, 4), new Rotation2d());
        Pose2d endPose = new Pose2d(new Translation2d(4, 4), new Rotation2d());

        List<XbotSwervePoint> swervePoints = routingCircle.generateSwervePoints(startPose, endPose);

        assertEquals(2, swervePoints.size());
        assertEquals(startPose.getTranslation(), swervePoints.get(0).getTranslation2d());
        assertEquals(endPose.getTranslation(), swervePoints.get(1).getTranslation2d());
    }

    @Test
    public void testGenerateSwervePointsWithIntersection() {
        Translation2d center = new Translation2d(0, 0);
        double radius = 2.0;
        ReefRoutingCircle routingCircle = new ReefRoutingCircle(center, radius);

        Pose2d startPose = new Pose2d(new Translation2d(-5, 0), new Rotation2d());

        Translation2d unitXVector = new Translation2d(1, 0);
        var rotatedUnitXVector = unitXVector.rotateBy(new Rotation2d(Degrees.of(10).in(Radians)));
        var scaledUnitVector = rotatedUnitXVector.times(routingCircle.getInnerCollisionCircleRadius()+0.1);

        Pose2d endPose = new Pose2d(scaledUnitVector, new Rotation2d());

        List<XbotSwervePoint> swervePoints = routingCircle.generateSwervePoints(startPose, endPose);

        assertTrue(swervePoints.size() > 2);
        assertEquals(startPose.getTranslation(), swervePoints.get(0).getTranslation2d());
        assertEquals(endPose.getTranslation(), swervePoints.get(swervePoints.size() - 1).getTranslation2d());
    }

    @Test
    public void testGenerateSwervePointsEdgeCase() {
        Translation2d center = new Translation2d(0, 0);
        double radius = 2.0;
        ReefRoutingCircle routingCircle = new ReefRoutingCircle(center, radius);

        Pose2d startPose = new Pose2d(new Translation2d(2, 0), new Rotation2d());
        Pose2d endPose = new Pose2d(new Translation2d(2, 2), new Rotation2d());

        List<XbotSwervePoint> swervePoints = routingCircle.generateSwervePoints(startPose, endPose);

        assertTrue(swervePoints.size() > 2);
        assertEquals(startPose.getTranslation(), swervePoints.get(0).getTranslation2d());
        assertEquals(endPose.getTranslation(), swervePoints.get(swervePoints.size() - 1).getTranslation2d());
    }


}