package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.arm_pivot.ArmPivotSubsystem;
import org.junit.Test;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.Assert.assertEquals;

public class ArmPivotAngleWrappingTest extends BaseCompetitionTest {


    @Test
    public void testAbsoluteEncoderSafeRange() {
        // Test #1: Absolute encoder position is in safe range, > maxPosition , < minPosition
        assertEquals(Degrees.of(62.5),
                ArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(270), false, 125));
        assertEquals(Degrees.of( 25),
                ArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(162), false, 125));
        assertEquals(Degrees.of( 96.25),
                ArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(18), false, 125));

        assertEquals(Degrees.of(31.25),
                ArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(360), false, 125));
        assertEquals(Degrees.of( 100),
                ArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(216), false, 125));
        assertEquals(Degrees.of( 62.5),
                ArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(108), false, 125));

        assertEquals(Degrees.of(37.5),
                ArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(90), false, 125));
//        assertEquals(Degrees.of( 100),
//                ArmPivotSubsystem.getArmAngle(0.75, 0.8,
//                        Degrees.of(216), false, 125));
//        assertEquals(Degrees.of( 62.5),
//                ArmPivotSubsystem.getArmAngle(0.75, 0.8,
//                        Degrees.of(108), false, 125));
    }

    @Test
    public void testAbsoluteEncoderUnsafeRange() {

        // Test #2: Absolute encoder position not in safe range and sensorHit
        assertEquals(Degrees.of(1.875),
                ArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(95.4), true, 125));
        assertEquals(Degrees.of(1.875),
                ArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(275.4), true, 125));

        // Test #3: Absolute encoder position not in safe range and sensor not hit
        assertEquals(Degrees.of(123.125),
                ArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(95.4), false, 125));

        assertEquals(Degrees.of(120.625),
                ArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(275.4), false, 125));
    }

}
