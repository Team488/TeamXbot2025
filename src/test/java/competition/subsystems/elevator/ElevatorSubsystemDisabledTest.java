package competition.subsystems.elevator;

import competition.BaseCompetitionTest;
import competition.electrical_contract.UnitTestContract2025;
import competition.injection.components.CompetitionTestComponent;
import competition.injection.components.DaggerCompetitionTestComponent;
import org.junit.Test;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class ElevatorSubsystemDisabledTest extends BaseCompetitionTest {
    @Override
    protected CompetitionTestComponent createDaggerComponent() {
        return (CompetitionTestComponent) DaggerCompetitionTestComponent
                .builder()
                .electricalContract(new UnitTestContract2025() {
                    @Override
                    public boolean isElevatorReady() {
                        return false;
                    }

                    @Override
                    public boolean isElevatorBottomSensorReady() {
                        return false;
                    }

                    @Override
                    public boolean isElevatorDistanceSensorReady() {
                        return false;
                    }
                })
                .build();
    }

    @Test
    public void testElevatorSubsystem() {
        var subsystem = getInjectorComponent().elevatorSubsystem();
        assertNotNull(subsystem);

        subsystem.refreshDataFrame();
        subsystem.periodic();

        assertNull(subsystem.masterMotor);

        assertTrue(Meters.of(0).isNear(subsystem.getCurrentValue(), 0.001));
        assertTrue(MetersPerSecond.of(0).isNear(subsystem.getCurrentVelocity(), 0.001));

        assertFalse(subsystem.isTouchingBottom());

        subsystem.markElevatorAsCalibratedAgainstLowerLimit();
    }
}
