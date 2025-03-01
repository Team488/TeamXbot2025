package competition.subsystems.drive.commands;

import competition.BaseCompetitionTest;
import org.junit.Test;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.drive.commands.DriveToBezierCurvesWithVisionCommand;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;
import java.util.Arrays;

public class DriveToCoralStationWithVisionCommandTest extends BaseCompetitionTest {
    Distance getDistanceFromCenterToOuterBumperX = Inches.of(18);

    @Test
    public void testTranslation() {
        var distanceToOuterBumerInMeters = getDistanceFromCenterToOuterBumperX.in(Meters);
        var coralStationPose = Landmarks.getCoralStationSectionPose(Landmarks.CoralStation.LEFT,
                Landmarks.CoralStationSection.MID);
        var distance = Math.sqrt(Math.pow(distanceToOuterBumerInMeters, 2.0) * 2.0);
        var deltaTranslation = new Translation2d(distance, coralStationPose.getRotation());
        var destinationTranslation = coralStationPose.getTranslation().plus(deltaTranslation);
        var destinationPose = new Pose2d(destinationTranslation, coralStationPose.getRotation());

        var expectedFinalX = coralStationPose.getX() + (coralStationPose.getRotation().getCos() * distance);
        var expectedFinalY = coralStationPose.getY() + (coralStationPose.getRotation().getSin() * distance);
        assertEquals(expectedFinalX, destinationPose.getX(), 0.001);
        assertEquals(expectedFinalY, destinationPose.getY(), 0.001);
        assertEquals(destinationPose.getRotation(), coralStationPose.getRotation());
    }

    @Test
    public void testAreListEqual() {
        var firstList = Arrays.asList(1, 3, 5, 7);
        var secondList = Arrays.asList(1, 3, 5);

        assertFalse(DriveToBezierCurvesWithVisionCommand.areListsEqual(firstList, secondList));
        secondList = Arrays.asList(1, 3, 5, 7);
        assertTrue(DriveToBezierCurvesWithVisionCommand.areListsEqual(firstList, secondList));
        secondList = Arrays.asList(1, 5, 3, 7);
        assertFalse(DriveToBezierCurvesWithVisionCommand.areListsEqual(firstList, secondList));
    }
}
