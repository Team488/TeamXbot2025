package competition.subsystems.deadwheel;

import xbot.common.controls.sensors.XEncoder;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.sensors.XEncoder.XEncoderFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class DeadWheelSubsystem extends BaseSubsystem {

    public final XEncoder leftEncoder;
    public final XEncoder rightEncoder;
    public final XEncoder frontEncoder;
    public final XEncoder rearEncoder;

    private final DoubleProperty wheelDiameterMeters;
    private final DoubleProperty pulsesPerRevolution;

    private Pose2d currentPose = new Pose2d();
    private double prevLeftDistance = 0;
    private double prevRightDistance = 0;
    private double prevFrontDistance = 0;
    private double prevRearDistance = 0;

    @Inject
    public DeadWheelSubsystem(XEncoderFactory encoderFactory, PropertyFactory propManager) {
        super();
        propManager.setPrefix(this);
        propManager.setDefaultLevel(Property.PropertyLevel.Important);
        this.wheelDiameterMeters = propManager.createPersistentProperty("wheelDiameterMeters", 0.032);
        this.pulsesPerRevolution = propManager.createPersistentProperty("pulsesPerRevolution", 500.0);

        double distancePerPulse = (Math.PI * wheelDiameterMeters.get()) / pulsesPerRevolution.get();

        leftEncoder = encoderFactory.create("LeftDeadwheelEncoder",
                21, 20, distancePerPulse, this.getName());
        rightEncoder = encoderFactory.create("RightDeadwheelEncoder",
                7, 8, distancePerPulse, this.getName());
        frontEncoder = encoderFactory.create("FrontDeadwheelEncoder",
                19, 18, distancePerPulse, this.getName());
        rearEncoder = encoderFactory.create("RearDeadwheelEncoder",
                5, 6, distancePerPulse, this.getName());

        leftEncoder.setInverted(true);
        rightEncoder.setInverted(false);
        frontEncoder.setInverted(true);
        rearEncoder.setInverted(false);

        dataFrameRefreshables.add(leftEncoder);
        dataFrameRefreshables.add(rightEncoder);
        dataFrameRefreshables.add(frontEncoder);
        dataFrameRefreshables.add(rearEncoder);
    }

    public Distance getLeftAdjustedDistance() {
        return Units.Meters.of(leftEncoder.getAdjustedDistance());
    }

    public Distance getRightAdjustedDistance() {
        return Units.Meters.of(rightEncoder.getAdjustedDistance());
    }

    public Distance getFrontAdjustedDistance() {
        return Units.Meters.of(frontEncoder.getAdjustedDistance());
    }

    public Distance getRearAdjustedDistance() {
        return Units.Meters.of(rearEncoder.getAdjustedDistance());
    }

    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

    public Pose2d getEstimatedPosition() {
        return currentPose;
    }

    public void update() {
        double distancePerPulse = (Math.PI * wheelDiameterMeters.get()) / pulsesPerRevolution.get();

        leftEncoder.setDistancePerPulseSupplier(() -> distancePerPulse);
        rightEncoder.setDistancePerPulseSupplier(() -> distancePerPulse);
        frontEncoder.setDistancePerPulseSupplier(() -> distancePerPulse);
        rearEncoder.setDistancePerPulseSupplier(() -> distancePerPulse);

        this.aKitLog.record("DeadWheelLeftAdjusted", leftEncoder.getAdjustedDistance());        
        this.aKitLog.record("DeadWheelRightAdjusted", rightEncoder.getAdjustedDistance());
        this.aKitLog.record("DeadWheelFrontAdjusted", frontEncoder.getAdjustedDistance());
        this.aKitLog.record("DeadWheelRearAdjusted", rearEncoder.getAdjustedDistance());
    }
}
