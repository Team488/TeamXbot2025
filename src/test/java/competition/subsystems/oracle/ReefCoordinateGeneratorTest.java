package competition.subsystems.oracle;

import competition.BaseCompetitionTest;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.Test;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.Assert.assertEquals;

public class ReefCoordinateGeneratorTest extends BaseCompetitionTest {

    @Test
    public void testEasyCases() {
        ReefCoordinateGenerator generator = getInjectorComponent().reefCoordinateGenerator();

        var pose = generator.getPoseRelativeToReefCenter(DriverStation.Alliance.Blue, Landmarks.ReefFace.FAR, Meters.of(4), Meters.of(2));
        assertEquals(Landmarks.BlueCenterOfReef.getX() + 4, pose.getX(), 0.001);
        assertEquals(Landmarks.BlueCenterOfReef.getY() + 2, pose.getY(), 0.001);

        pose = generator.getPoseRelativeToReefCenter(DriverStation.Alliance.Blue, Landmarks.ReefFace.CLOSE, Meters.of(4), Meters.of(2));
        assertEquals(Landmarks.BlueCenterOfReef.getX() - 4, pose.getX(), 0.001);
        assertEquals(Landmarks.BlueCenterOfReef.getY() - 2, pose.getY(), 0.001);
    }

    @Test
    public void testEasyBranchAndFaceCases() {
        ReefCoordinateGenerator generator = getInjectorComponent().reefCoordinateGenerator();

        var pose = generator.getPoseRelativeToReefFaceAndBranch(
                DriverStation.Alliance.Blue, Landmarks.ReefFace.FAR, Landmarks.Branch.A, Meters.of(4), Meters.of(2));
        assertEquals(Landmarks.BlueCenterOfReef.getX() + Landmarks.reefCenterToFace.in(Meters) + 4, pose.getX(), 0.001);
        assertEquals(Landmarks.BlueCenterOfReef.getY() + Landmarks.reefBranchHorizontalOffsetForBranchTypeA.in(Meters) + 2, pose.getY(), 0.001);

        pose = generator.getPoseRelativeToReefFaceAndBranch(
                DriverStation.Alliance.Blue, Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Meters.of(4), Meters.of(2));
        assertEquals(Landmarks.BlueCenterOfReef.getX() - Landmarks.reefCenterToFace.in(Meters) - 4, pose.getX(), 0.001);
        assertEquals(Landmarks.BlueCenterOfReef.getY() - Landmarks.reefBranchHorizontalOffsetForBranchTypeA.in(Meters) - 2, pose.getY(), 0.001);
    }

    // Not an active test, just a way of quickly checking hardcoded values against the generated values to see if they are in the
    // same ballpark.
    //@Test
    public void compareHardcodedToGenerated() {
        ReefCoordinateGenerator generator = getInjectorComponent().reefCoordinateGenerator();
        double robotWithInMeters = getInjectorComponent().electricalContract().getDistanceFromCenterToOuterBumperX().in(Meters);

        var pose = generator.getPoseRelativeToReefFaceAndBranch(
                DriverStation.Alliance.Blue, Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Meters.of(robotWithInMeters), Meters.zero()).getTranslation();
        var actual = Landmarks.BlueCloseBranchA.getTranslation();
    }
}
