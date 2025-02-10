package competition.subsystems.oracle;

import competition.BaseCompetitionTest;
import competition.subsystems.pose.Landmarks;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class VirtualReefFaceTest extends BaseCompetitionTest {

    @Test
    public void testEmpty() {
        VirtualReefFace face = new VirtualReefFace(Landmarks.ReefFace.CLOSE, 1, Landmarks.ReefAlgae.LOW);

        var levels = new Landmarks.CoralLevel[]{
                Landmarks.CoralLevel.ONE,
                Landmarks.CoralLevel.TWO,
                Landmarks.CoralLevel.THREE,
                Landmarks.CoralLevel.FOUR};

        int count = face.getCountOfLocationsMatchingFilters(
                CoralState.Absent, false,
                levels);

        assertEquals(8, count);

        count = face.getCountOfLocationsMatchingFilters(
                CoralState.Absent, true,
                levels);

        assertEquals(4, count);

        count = face.getCountOfLocationsMatchingFilters(
                CoralState.Unavailable, false,
                levels);

        assertEquals(0, count);

        count = face.getCountOfLocationsMatchingFilters(
                CoralState.Unavailable, true,
                levels);

        assertEquals(0, count);

        count = face.getCountOfLocationsMatchingFilters(
                CoralState.Present, false,
                levels);

        assertEquals(0, count);

        count = face.getCountOfLocationsMatchingFilters(
                CoralState.Present, true,
                levels);

        assertEquals(0, count);

        var levelsOneAndTwo = new Landmarks.CoralLevel[]{
                Landmarks.CoralLevel.ONE,
                Landmarks.CoralLevel.TWO};

        count = face.getCountOfLocationsMatchingFilters(
                CoralState.Absent, false,
                levelsOneAndTwo);

        assertEquals(4, count);

        count = face.getCountOfLocationsMatchingFilters(
                CoralState.Absent, true,
                levelsOneAndTwo);

        assertEquals(2, count);
    }

    @Test
    public void testMarkingLocationsPresent() {
        VirtualReefFace face = new VirtualReefFace(Landmarks.ReefFace.CLOSE, 1, Landmarks.ReefAlgae.LOW);

        var levels = new Landmarks.CoralLevel[]{
                Landmarks.CoralLevel.ONE,
                Landmarks.CoralLevel.TWO,
                Landmarks.CoralLevel.THREE,
                Landmarks.CoralLevel.FOUR};

        int count = face.getCountOfLocationsMatchingFilters(
                CoralState.Absent, false,
                levels);

        assertEquals(8, count);

        face.setCoralState(
                new ReefFaceScoringLocation(Landmarks.Branch.A, Landmarks.CoralLevel.ONE), CoralState.Present);

        count = face.getCountOfLocationsMatchingFilters(
                CoralState.Absent, false,
                levels);

        assertEquals(7, count);
    }
}
