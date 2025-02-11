package competition.subsystems.neotrellis;

import competition.BaseCompetitionTest;
import competition.operator_interface.NeoTrellisSubsystem;
import competition.subsystems.oracle.ScoringQueue;
import competition.subsystems.pose.Landmarks;
import org.junit.Test;
import xbot.common.controls.sensors.mock_adapters.MockJoystick;

import static org.junit.Assert.assertEquals;

public class NeoTrellisSubsystemTest extends BaseCompetitionTest {

    private NeoTrellisSubsystem neoSubsystem;
    private MockJoystick neoTrellis;
    private ScoringQueue scoringQueue;

    @Override
    public void setUp() {
        super.setUp();
        neoSubsystem = getInjectorComponent().neoTrellisSubsystem();
        neoTrellis = (MockJoystick)getInjectorComponent().operatorInterface().neoTrellis;
        scoringQueue = getInjectorComponent().scoringQueue();
        scoringQueue.clearQueue();
    }

    @Test
    public void testNothing() {
        assertEquals(0, scoringQueue.getQueueSize());
        neoSubsystem.periodic();
        assertEquals(0, scoringQueue.getQueueSize());
    }

    @Test
    public void testSingleButtons() {
        assertEquals(0, scoringQueue.getQueueSize());
        neoTrellis.pressButton(neoSubsystem.getNeoTrellisButtonIndex(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A));
        neoSubsystem.periodic();
        assertEquals(0, scoringQueue.getQueueSize());
    }

    @Test
    public void testButtonCombo() {
        assertEquals(0, scoringQueue.getQueueSize());
        neoTrellis.pressButton(neoSubsystem.getNeoTrellisButtonIndex(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A));
        neoTrellis.pressButton(neoSubsystem.getNeoTrellisButtonIndex(Landmarks.CoralLevel.FOUR));
        neoSubsystem.periodic();
        assertEquals(1, scoringQueue.getQueueSize());
    }
}
