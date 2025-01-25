package competition.simulation.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ArmSimConstants {
    // TODO: this is all from the wpilib example but doesn't reflect our real math
    public static final double armReduction = 200;
    public static final Mass armMass = Kilogram.of(8.0); // Kilograms
    public static final Distance armLength = Meters.of(0.5);
    public static final Angle minAngleRads = Degrees.of(-45);
    public static final Angle maxAngleRads = Degrees.of(125 - 45);
    public static final Angle startingAngle = minAngleRads;
    public static final Angle armEncoderAnglePerRotation = Degrees.of(0.1);
}
