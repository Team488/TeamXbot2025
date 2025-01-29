package competition.simulation.elevator;

public class ElevatorSimConstants {
    // These are are all placeholders
    public static final double elevatorGearing = 10;
    public static final double carriageMass = 4;
    public static final double elevatorDrumRadius = 0.0508;
    public static final double minElevatorHeightMeters = 0;
    public static final double maxElevatorHeightMeters = 1.25;
    public static final double elevatorBottomSensorTriggerHeight = 0.01;
    public static final double rotationsPerMeterHeight = 1923; // arbitrary big number
    // this is random to mimic the motor not being zeroed when the robot turns on so the
    // code will have to handle the calibration thereof
    public static final double rotationsAtZero = Math.random() * 1000;
}
