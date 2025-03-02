package competition.subsystems.lights;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

import competition.BaseCompetitionTest;
import competition.subsystems.lights.LightSubsystem.LightsStateMessage;

public class LightSubsystemTest extends BaseCompetitionTest {
    @Test
    public void testSendState() {
        var subsystem = getInjectorComponent().lightSubsystem();
        assertNotNull(subsystem);

        subsystem.refreshDataFrame();
        subsystem.periodic();

        assertEquals(subsystem.dioInt.getDIOInt(), LightsStateMessage.RobotDisabledDefault.getValue());
    }

    @Test
    public void testDioInt() {
        var subsystem = getInjectorComponent().lightSubsystem();
        assertNotNull(subsystem);

        for(int i = 0; i < (Math.pow(2, LightSubsystem.numBits)); i++) {
            subsystem.dioInt.setDIOInt(i);
            subsystem.refreshDataFrame();
            assertEquals(subsystem.dioInt.getDIOInt(), i);
        }
    }
}
