package competition.subsystems.oracle;

import competition.BaseCompetitionTest;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class VirtualReefTest extends BaseCompetitionTest {

    @Test
    public void simpleTest() {
        VirtualReef reef = new VirtualReef(DriverStation.Alliance.Blue);

        var levels = new Landmarks.CoralLevel[] {
                Landmarks.CoralLevel.ONE,
                Landmarks.CoralLevel.TWO,
                Landmarks.CoralLevel.THREE,
                Landmarks.CoralLevel.FOUR
        };

        int count = reef.getCountOfLocationsMatchingFilters(CoralState.Absent, false, levels);
        assertEquals(48, count);

        count = reef.getCountOfLocationsMatchingFilters(CoralState.Absent, true, levels);
        assertEquals(30, count);

        reef.setCoralState(Landmarks.ReefFace.CLOSE,
                new ReefFaceScoringLocation(Landmarks.Branch.A, Landmarks.CoralLevel.ONE), CoralState.Present);

        count = reef.getCountOfLocationsMatchingFilters(CoralState.Absent, false, levels);
        assertEquals(47, count);

        reef.markFaceAsUnavailable(Landmarks.ReefFace.CLOSE);
        count = reef.getCountOfLocationsMatchingFilters(CoralState.Absent, false, levels);
        assertEquals(40, count);
    }
}
