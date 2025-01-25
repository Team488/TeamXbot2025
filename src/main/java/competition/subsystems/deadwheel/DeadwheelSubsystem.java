package competition.subsystems.deadwheel;

import xbot.common.controls.sensors.XEncoder;
import xbot.common.controls.sensors.XEncoderFactory;
import xbot.common.math.WrappedRotation2d;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.BaseSubsystem;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class DeadwheelSubsystem extends BaseSubsystem {

    private final XEncoder leftEncoder;
    private final XEncoder rightEncoder;
    private final XEncoder frontEncoder;

    private final double wheelDiameterMeters = 0.032; // 32 mm
    private final int pulsesPerRevolution = 2000;
    private final double distancePerPulse = (Math.PI * wheelDiameterMeters) / pulsesPerRevolution;

    @Inject
    public DeadwheelSubsystem(XEncoderFactory encoderFactory, PropertyFactory propFactory) {
        leftEncoder = encoderFactory.create("LeftDeadwheelEncoder");
        rightEncoder = encoderFactory.create("RightDeadwheelEncoder");
        frontEncoder = encoderFactory.create("FrontDeadwheelEncoder");

        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
        frontEncoder.setDistancePerPulse(distancePerPulse);
    }

    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    public double getFrontDistance() {
        return frontEncoder.getDistance();
    }

    public double getLeftRate() {
        return leftEncoder.getRate();
    }

    public double getRightRate() {
        return rightEncoder.getRate();
    }

    public double getFrontRate() {
        return frontEncoder.getRate();
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        frontEncoder.reset();
    }
}
