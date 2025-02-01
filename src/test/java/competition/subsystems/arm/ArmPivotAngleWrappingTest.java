package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;

import org.junit.Test;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.Assert.assertEquals;

public class ArmPivotAngleWrappingTest extends BaseCompetitionTest {


    @Test
    public void testAbsoluteEncoderSafeRange() {
        // Test #1: Absolute encoder position is in safe range, > maxPosition , < minPosition
        assertEquals(Degrees.of(62.5),
                CoralArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(270), false, 125));
        assertEquals(Degrees.of( 25),
                CoralArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(162), false, 125));
        assertEquals(Degrees.of( 96.25),
                CoralArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(18), false, 125));

        assertEquals(Degrees.of(31.25),
                CoralArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(360), false, 125));
        assertEquals(Degrees.of( 100),
                CoralArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(216), false, 125));
        assertEquals(Degrees.of( 62.5),
                CoralArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(108), false, 125));

        // Special Case: maxPosition < minPosition
        assertEquals(Degrees.of(32.5),
                CoralArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(90), false, 125));
        assertEquals(Degrees.of(95),
                CoralArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(270), false, 125));
        assertEquals(Degrees.of(63.75),
                CoralArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(180), false, 125));
    }

    @Test
    public void testAbsoluteEncoderUnsafeRange() {
        // Test #2: Absolute encoder position not in safe range and sensorHit (Arm is very low)
        assertEquals(Degrees.of(1.875),
                CoralArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(95.4), true, 125));
        assertEquals(Degrees.of(1.875),
                CoralArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(275.4), true, 125));
        // Special Case: maxPosition < minPosition
        // testing when absEncoderPosition is greater than minPosition but also when it is less than maxPosition
        assertEquals(Degrees.of(0.625),
                CoralArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(358.2), true, 125));
        assertEquals(Degrees.of(1.25),
                CoralArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(360), true, 125));
        assertEquals(Degrees.of(3.75),
                CoralArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(7.2), true, 125));


        // Test #3: Absolute encoder position not in safe range and sensor not hit (Arm is very high)
        assertEquals(Degrees.of(123.125),
                CoralArmPivotSubsystem.getArmAngle(0.25, 0.28,
                        Degrees.of(95.4), false, 125));
        assertEquals(Degrees.of(120.625),
                CoralArmPivotSubsystem.getArmAngle(0.75, 0.8,
                        Degrees.of(275.4), false, 125));
        // Special Case: maxPosition < minPosition
        // testing when absEncoderPosition is greater than minPosition but also when it is less than maxPosition
        assertEquals(Degrees.of(120),
                CoralArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(360), false, 125));
        assertEquals(Degrees.of(122.5),
                CoralArmPivotSubsystem.getArmAngle(0.99, 0.04,
                        Degrees.of(7.2), false, 125));

    }

}
