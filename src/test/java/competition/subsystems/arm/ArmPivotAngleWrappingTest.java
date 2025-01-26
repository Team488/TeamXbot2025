package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.arm_pivot.ArmPivotSubsystem;
import org.junit.Test;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.Assert.assertEquals;

public class ArmPivotAngleWrappingTest extends BaseCompetitionTest {
    @Test
    public void test() {
        // Test #1: Absolute encoder position is in safe range, > maxPosition , < minPosition
        assertEquals(Degrees.of(62.5),
                ArmPivotSubsystem.getArmAngleTest(0.25, 0.28,
                        0.75, false, 125));
        assertEquals(Degrees.of( 25),
                ArmPivotSubsystem.getArmAngleTest(0.25, 0.28,
                        0.45, false, 125));
        assertEquals(Degrees.of( 96.25),
                ArmPivotSubsystem.getArmAngleTest(0.25, 0.28,
                        0.05, false, 125));
        // Test #2: Absolute encoder position not in safe range and sensorHit
        assertEquals(Degrees.of(1.875),
                ArmPivotSubsystem.getArmAngleTest(0.25, 0.28,
                        0.265, true, 125));
        // Test #3: Absolute encoder position not in safe range and sensor not hit
        assertEquals(Degrees.of(123.125),
                ArmPivotSubsystem.getArmAngleTest(0.25, 0.28,
                        0.265, false, 125));

        // Same test but with new minPosition and maxPosition
        assertEquals(Degrees.of(31.25),
                ArmPivotSubsystem.getArmAngleTest(0.75, 0.8,
                        1, false, 125));
        assertEquals(Degrees.of( 100),
                ArmPivotSubsystem.getArmAngleTest(0.75, 0.8,
                        0.6, false, 125));
        assertEquals(Degrees.of( 62.5),
                ArmPivotSubsystem.getArmAngleTest(0.75, 0.8,
                        0.3, false, 125));

        assertEquals(Degrees.of(1.875),
                ArmPivotSubsystem.getArmAngleTest(0.75, 0.8,
                        0.765, true, 125));

        assertEquals(Degrees.of(120.625),
                ArmPivotSubsystem.getArmAngleTest(0.75, 0.8,
                        0.765, false, 125));
    }
}
