package competition.motion;

import competition.BaseCompetitionTest;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class TrapezoidProfileManagerTest extends BaseCompetitionTest {

    private TrapezoidProfileManager manager;

    @Override
    public void setUp() {
        super.setUp();
        manager = getInjectorComponent().trapezoidProfileManagerFactory()
                .create("test", 1, 1, 0);
    }

    @Test
    public void setTargetPosition() {
        manager.setTargetPosition(1, 0, 0);
        assertEquals(1, manager.goalState.position, 0.001);
        assertEquals(0, manager.goalState.velocity, 0.001);
        assertEquals(0, manager.initialState.position, 0.001);
        assertEquals(0, manager.initialState.velocity, 0.001);
    }

    @Test
    public void getRecommendedPositionForTime() {
        var recommendation = manager.getRecommendedPositionForTime();
        assertEquals(0, recommendation, 0.001);
    }

    @Test
    public void endToEndTest() {
        manager.setTargetPosition(1, 0, 0);
        var recommendation = manager.getRecommendedPositionForTime();
        assertEquals(0, recommendation, 0.001);
        timer.advanceTimeInSecondsBy(1);
        recommendation = manager.getRecommendedPositionForTime();
        assertEquals(0.5, recommendation, 0.001);

        manager.setTargetPosition(2, 1, 0.5);
        recommendation = manager.getRecommendedPositionForTime();
        assertEquals(1, recommendation, 0.001);
        timer.advanceTimeInSecondsBy(1);
        recommendation = manager.getRecommendedPositionForTime();
        assertEquals(1.8047, recommendation, 0.001);
    }
}