@Singleton
public class DeadWheelSubsystem extends BaseSubsystem {

    private final XEncoder leftEncoder;
    private final XEncoder rightEncoder;
    private final XEncoder frontEncoder;
    private final XEncoder rearEncoder;
    private final double trackWidth;

    private final double wheelDiameterMeters;
    private final int pulsesPerRevolution;
    private final double distancePerPulse;

    private Pose2d currentPose = new Pose2d();
    private double prevLeftDistance = 0;
    private double prevRightDistance = 0;
    private double prevFrontDistance = 0;
    private double prevRearDistance = 0;

    @Inject
    public DeadWheelSubsystem(XEncoderFactory encoderFactory, PropertyFactory propFactory,
                              @Named("TrackWidth") double trackWidth, 
                              @Named("WheelDiameterMeters") double wheelDiameterMeters, 
                              @Named("PulsesPerRevolution") int pulsesPerRevolution) {
        leftEncoder = encoderFactory.create("LeftDeadwheelEncoder");
        rightEncoder = encoderFactory.create("RightDeadwheelEncoder");
        frontEncoder = encoderFactory.create("FrontDeadwheelEncoder");
        rearEncoder = encoderFactory.create("RearDeadwheelEncoder");

        this.wheelDiameterMeters = wheelDiameterMeters;
        this.pulsesPerRevolution = pulsesPerRevolution;
        this.trackWidth = trackWidth;
        this.distancePerPulse = (Math.PI * wheelDiameterMeters) / pulsesPerRevolution;

        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
        frontEncoder.setDistancePerPulse(distancePerPulse);
        rearEncoder.setDistancePerPulse(distancePerPulse);
    }

    public Pose2d updateOdometry() {
        double leftDistance = leftEncoder.getDistance();
        double rightDistance = rightEncoder.getDistance();
        double frontDistance = frontEncoder.getDistance();
        double rearDistance = rearEncoder.getDistance();

        double d_left = leftDistance - prevLeftDistance;
        double d_right = rightDistance - prevRightDistance;
        double d_front = frontDistance - prevFrontDistance;
        double d_rear = rearDistance - prevRearDistance;

        double d_theta_x = (d_right - d_left) / trackWidth;
        double d_theta_y = (d_front - d_rear) / trackWidth;
        double d_theta = (d_theta_x + d_theta_y) / 2.0;

        double avg_distance_x = (d_left + d_right) / 2.0;
        double avg_distance_y = (d_front + d_rear) / 2.0;

        double d_x = avg_distance_x * Math.cos(currentPose.getRotation().getRadians());
        double d_y = avg_distance_y * Math.sin(currentPose.getRotation().getRadians());

        prevLeftDistance = leftDistance;
        prevRightDistance = rightDistance;
        prevFrontDistance = frontDistance;
        prevRearDistance = rearDistance;

        currentPose = new Pose2d(
            currentPose.getX() + d_x,
            currentPose.getY() + d_y,
            currentPose.getRotation().plus(new Rotation2d(d_theta))
        );

        return currentPose;
    }
}
