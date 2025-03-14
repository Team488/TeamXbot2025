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
        var autonomous = getInjectorComponent().autonomousCommandSelector();
        var command = getInjectorComponent().alignToNearestCoralStationCommand();
        assertNotNull(subsystem);

        autonomous.setCurrentAutonomousCommand(getInjectorComponent().emergencyAutonomousCommand());

        subsystem.refreshDataFrame();
        subsystem.periodic();

        assertEquals(LightsStateMessage.RobotDisabledDefault.getValue(), subsystem.dioInt.getDIOInt());

        autonomous.setCurrentAutonomousCommand(command);

        subsystem.refreshDataFrame();
        subsystem.periodic();

        assertEquals(LightsStateMessage.RobotDisabledAuto.getValue(), subsystem.dioInt.getDIOInt());
    }

    @Test
    public void testDioInt() {
        var subsystem = getInjectorComponent().lightSubsystem();
        assertNotNull(subsystem);

        for(int i = 0; i < (Math.pow(2, 4)); i++) {
            subsystem.dioInt.setDIOInt(i);
            subsystem.refreshDataFrame();
            assertEquals(subsystem.dioInt.getDIOInt(), i);
        }
    }
}
