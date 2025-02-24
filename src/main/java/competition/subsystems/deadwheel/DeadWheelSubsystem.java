package competition.subsystems.deadwheel;
import xbot.common.controls.sensors.XEncoder;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.sensors.XEncoder.XEncoderFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class DeadWheelSubsystem extends BaseSubsystem {

    private final XEncoder leftEncoder;
    private final XEncoder rightEncoder;
    private final XEncoder frontEncoder;
    private final XEncoder rearEncoder;
    private final DoubleProperty trackWidth;


    private Pose2d currentPose = new Pose2d();
    private double prevLeftDistance = 0;
    private double prevRightDistance = 0;
    private double prevFrontDistance = 0;
    private double prevRearDistance = 0;

    @Inject
    public DeadWheelSubsystem(XEncoderFactory encoderFactory, PropertyFactory propManager)
    {
        super();
        propManager.setPrefix(this);
        propManager.setDefaultLevel(Property.PropertyLevel.Important);
        DoubleProperty wheelDiameterMeters = propManager.createPersistentProperty("wheelDiameterMeters", 0.032);
        DoubleProperty pulsesPerRevolution = propManager.createPersistentProperty("pulsesPerRevolution", 2000.0);

        this.trackWidth = propManager.createPersistentProperty("TrackWidth", 0.5);

        double distancePerPulse = (Math.PI * wheelDiameterMeters.get()) / pulsesPerRevolution.get();

        leftEncoder = encoderFactory.create("LeftDeadwheelEncoder",
                21,20, distancePerPulse, "LeftDeadWheel");
        rightEncoder = encoderFactory.create("RightDeadwheelEncoder",
                7,8, distancePerPulse, "RightDeadWheel");
        frontEncoder = encoderFactory.create("FrontDeadwheelEncoder",
                19,18, distancePerPulse, "FrontDeadWheel");
        rearEncoder = encoderFactory.create("RearDeadwheelEncoder",
                5,6, distancePerPulse, "RearDeadWheel");

    }


    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

    public Pose2d getEstimatedPosition() {
        return currentPose;
    }

    public void update() {
        double leftDistance = leftEncoder.getAdjustedDistance();
        double rightDistance = rightEncoder.getAdjustedDistance();
        double frontDistance = frontEncoder.getAdjustedDistance();
        double rearDistance = rearEncoder.getAdjustedDistance();

        double d_left = leftDistance - prevLeftDistance;
        double d_right = rightDistance - prevRightDistance;
        double d_front = frontDistance - prevFrontDistance;
        double d_rear = rearDistance - prevRearDistance;

        double d_theta_x = (d_right - d_left) / trackWidth.get();
        double d_theta_y = (d_front - d_rear) / trackWidth.get();
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
    }
}
