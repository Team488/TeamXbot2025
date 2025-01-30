import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class DeadwheelSubsystem extends BaseSubsystem {

    private final XEncoder leftEncoder;
    private final XEncoder rightEncoder;
    private final XEncoder frontEncoder;
    private final double trackWidth;
    private final double frontWheelOffset;

    private final double wheelDiameterMeters;
    private final int pulsesPerRevolution;
    private final double distancePerPulse;

    private Pose2d currentPose = new Pose2d();
    private double prevLeftDistance = 0;
    private double prevRightDistance = 0;
    private double prevFrontDistance = 0;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    @Inject
    public DeadwheelSubsystem(XEncoderFactory encoderFactory, PropertyFactory propFactory, 
                              @Named("TrackWidth") double trackWidth, 
                              @Named("WheelDiameterMeters") double wheelDiameterMeters, 
                              @Named("PulsesPerRevolution") int pulsesPerRevolution, 
                              @Named("FrontWheelOffset") double frontWheelOffset,
                              SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
        this.odometry = new SwerveDriveOdometry(kinematics, currentPose.getRotation());

        leftEncoder = encoderFactory.create("LeftDeadwheelEncoder");
        rightEncoder = encoderFactory.create("RightDeadwheelEncoder");
        frontEncoder = encoderFactory.create("FrontDeadwheelEncoder");

        this.wheelDiameterMeters = wheelDiameterMeters;
        this.pulsesPerRevolution = pulsesPerRevolution;
        this.trackWidth = trackWidth;
        this.frontWheelOffset = frontWheelOffset;
        this.distancePerPulse = (Math.PI * wheelDiameterMeters) / pulsesPerRevolution;

        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
        frontEncoder.setDistancePerPulse(distancePerPulse);
    }

    public Pose2d updateOdometry() {
        double leftDistance = leftEncoder.getDistance();
        double rightDistance = rightEncoder.getDistance();
        double frontDistance = frontEncoder.getDistance();

        double d_left = leftDistance - prevLeftDistance;
        double d_right = rightDistance - prevRightDistance;
        double d_front = frontDistance - prevFrontDistance;

        double d_theta = (d_right - d_left) / trackWidth;
        double avg_distance = (d_left + d_right) / 2.0;

        double d_x = avg_distance * Math.cos(currentPose.getRotation().getRadians()) + d_front * Math.sin(currentPose.getRotation().getRadians()) - frontWheelOffset * d_theta * Math.sin(currentPose.getRotation().getRadians());
        double d_y = avg_distance * Math.sin(currentPose.getRotation().getRadians()) - d_front * Math.cos(currentPose.getRotation().getRadians()) - frontWheelOffset * d_theta * Math.cos(currentPose.getRotation().getRadians());

        Twist2d twist = new Twist2d(d_x, d_y, d_theta);

        currentPose = currentPose.exp(twist);

        prevLeftDistance = leftDistance;
        prevRightDistance = rightDistance;
        prevFrontDistance = frontDistance;

        odometry.update(currentPose.getRotation(), new SwerveModulePosition[] {
            new SwerveModulePosition(leftDistance, currentPose.getRotation()),
            new SwerveModulePosition(rightDistance, currentPose.getRotation()),
            new SwerveModulePosition(frontDistance + frontWheelOffset, currentPose.getRotation())
        });

        return currentPose;
    }
}
