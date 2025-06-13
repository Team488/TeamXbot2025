package competition.simulation.coral_arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class CoralArmSimConstants {
    // TODO: this is all from the wpilib example but doesn't reflect our real math
    public static final double armReduction = 50;
    public static final Mass armMass = Kilogram.of(8.0); // Kilograms
    public static final Distance armLength = Meters.of(0.5);

    // the frame of reference for these angles is 0' right, 90' up, 180' left, 270' down
    // so we have 225 as the starting angle which is 0' in arm relative terms
    public static final Angle armEncoderAnglePerRotation = Degrees.of(5.790);
    public static final Angle maxAngleRads = Degrees.of(245);
    public static final Angle angleAtRobotZero = maxAngleRads;
    public static final Angle startingAngle = maxAngleRads;
    // subtrack the range of motion from the starting angle to get the min angle
    public static final Angle minAngleRads = startingAngle.minus(Degrees.of(230));
}
