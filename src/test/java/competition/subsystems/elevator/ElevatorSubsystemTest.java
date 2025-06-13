package competition.subsystems.elevator;

import competition.BaseCompetitionTest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.junit.Test;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class ElevatorSubsystemTest extends BaseCompetitionTest {
    @Test
    public void testElevatorSubsystem() {
        var subsystem = getInjectorComponent().elevatorSubsystem();
        assertNotNull(subsystem);

        subsystem.refreshDataFrame();
        subsystem.periodic();

        assertNotNull(subsystem.masterMotor);

        assertTrue(Meters.of(0).isNear(subsystem.getCurrentValue(), 0.001));
        assertTrue(MetersPerSecond.of(0).isNear(subsystem.getCurrentVelocity(), 0.001));
    }

    @Test
    public void testElevatorSubsystemGetSysIdCommands() {
        var subsystem = getInjectorComponent().elevatorSubsystem();
        assertNotNull(subsystem);

        assertNotNull(subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        assertNotNull(subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    }
}
