package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.simulation.coral_arm.CoralArmSimulator;

import competition.subsystems.stinger_arm.StingerArmSubsystem;
import org.junit.Test;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.Assert.assertEquals;

public class CoralArmAngleSimulatorTest extends BaseCompetitionTest {


    @Test
    public void testAbsoluteEncoderSafeRange() {
        assertEquals(Degrees.of(50),
                StingerArmSubsystem.getArmAngle(0.25, 0.3,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(50),
                                0.25, 0.3),
                        false, 125)
                );
        assertEquals(Degrees.of(90),
                StingerArmSubsystem.getArmAngle(0.25, 0.3,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(90),
                                0.25, 0.3),
                        false, 125)
        );

        assertEquals(Degrees.of(50),
                StingerArmSubsystem.getArmAngle(0.99, 0.04,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(50),
                                0.99, 0.04),
                        false, 125)
        );
        assertEquals(Degrees.of(90),
                StingerArmSubsystem.getArmAngle(0.99, 0.04,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(90),
                                0.99, 0.04),
                        false, 125)
        );
    }

    @Test
    public void testAbsoluteEncoderUnSafeRange() {
        assertEquals(Degrees.of(5),
                StingerArmSubsystem.getArmAngle(0.25, 0.3,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(5),
                                0.25, 0.3),
                        true, 125)
        );
        assertEquals(Degrees.of(2.5),
                StingerArmSubsystem.getArmAngle(0.25, 0.3,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(2.5),
                                0.25, 0.3),
                        true, 125)
        );

        assertEquals(Degrees.of(5),
                StingerArmSubsystem.getArmAngle(0.99, 0.04,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(5),
                                0.99, 0.04),
                        true, 125)
        );

        assertEquals(Degrees.of(120),
                StingerArmSubsystem.getArmAngle(0.25, 0.3,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(120),
                                0.25, 0.3),
                        false, 125)
        );
        assertEquals(Degrees.of(122.5),
                StingerArmSubsystem.getArmAngle(0.25, 0.3,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(122.5),
                                0.25, 0.3),
                        false, 125)
        );

        assertEquals(Degrees.of(120),
                StingerArmSubsystem.getArmAngle(0.99, 0.04,
                        CoralArmSimulator.getAbsoluteEncoderPosition(Degrees.of(120),
                                0.99, 0.04),
                        false, 125)
        );
    }

}