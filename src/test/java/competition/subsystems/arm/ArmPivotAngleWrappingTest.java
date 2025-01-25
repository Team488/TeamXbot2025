package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.arm_pivot.ArmPivotSubsystem;
import org.junit.Test;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.Assert.assertEquals;

public class ArmPivotAngleWrappingTest extends BaseCompetitionTest {
    @Test
    public void test() {
        // Tests #1: Absolute encoder position is in safe range, > maxPosition , < minPosition
        assertEquals(Degrees.of(75),
                ArmPivotSubsystem.getArmAngleTest(0.25, 0.28,
                        0.75, false, 125));
        assertEquals(Degrees.of( 25),
                ArmPivotSubsystem.getArmAngleTest(0.25, 0.28,
                        0.45, false, 125));
        assertEquals(Degrees.of( 100),
                ArmPivotSubsystem.getArmAngleTest(0.25, 0.28,
                        0.05, false, 125));
    }
}
