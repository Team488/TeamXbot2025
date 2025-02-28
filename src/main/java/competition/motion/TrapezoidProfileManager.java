package competition.motion;

import static edu.wpi.first.units.Units.Seconds;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import dagger.assisted.Assisted;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutTime;
import org.checkerframework.checker.units.qual.A;
import xbot.common.advantage.AKitLogger;
import xbot.common.controls.sensors.XTimer;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public class TrapezoidProfileManager {
    protected final Logger log;
    private final AKitLogger aKitLog;

    @AssistedFactory
    public abstract static class Factory {
        public abstract TrapezoidProfileManager create(
                @Assisted String name,
                @Assisted("defaultMaxVelocity") double defaultMaxVelocity,
                @Assisted("defaultMaxAcceleration") double defaultMaxAcceleration,
                @Assisted("defaultMaxGap") double defaultMaxGap,
                @Assisted("initialPosition") double initialPosition);
    }

    final DoubleProperty maxVelocity;
    final DoubleProperty maxAcceleration;
    final DoubleProperty maxGap;
    
    TrapezoidProfile profile;
    TrapezoidProfile.Constraints constraints;
    TrapezoidProfile.State initialState;
    TrapezoidProfile.State goalState;
    MutTime profileStartTime = Seconds.mutable(0);
    double previousSetpoint = 0;
    MutTime previousSetpointTime = Seconds.mutable(-10000);

    final GlobalSafeSpeedsManager globalSafeSpeedsManager;
    double multiplier;

    @AssistedInject
    public TrapezoidProfileManager(
            @Assisted String name,
            PropertyFactory pf,
            GlobalSafeSpeedsManager globalSafeSpeedsManager,
            @Assisted("defaultMaxVelocity") double defaultMaxVelocity,
            @Assisted("defaultMaxAcceleration") double defaultMaxAcceleration,
            @Assisted("defaultMaxGap") double defaultMaxGap,
            @Assisted("initialPosition") double initialPosition) {
        pf.setPrefix(name);
        log = LogManager.getLogger(name + ": TrapezoidProfileManager");
        aKitLog = new AKitLogger(pf.getPrefix());
        aKitLog.setLogLevel(AKitLogger.LogLevel.DEBUG);
        maxVelocity = pf.createPersistentProperty("maxVelocity", defaultMaxVelocity);
        maxAcceleration = pf.createPersistentProperty("maxAcceleration", defaultMaxAcceleration);
        maxGap = pf.createPersistentProperty("maxGap", defaultMaxGap);
        constraints = new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
        profile = new TrapezoidProfile(constraints);
        // initialize states to current value
        initialState = new TrapezoidProfile.State(initialPosition, 0);
        goalState = new TrapezoidProfile.State(initialPosition, 0);

        this.globalSafeSpeedsManager = globalSafeSpeedsManager;
    }

    public void resetState(double currentValue, double currentVelocity) {
        initialState = new TrapezoidProfile.State(currentValue, currentVelocity);
        goalState = new TrapezoidProfile.State(currentValue, 0);
        previousSetpoint = currentValue;
        profileStartTime.mut_replace(XTimer.getFPGATimestampTime().minus(Seconds.of(0.02)));
    }

    private double getMaxVelocity() {
        double effectiveVelocity = maxVelocity.get() * multiplier;
        aKitLog.record("EffectiveVelocity", effectiveVelocity);
        return effectiveVelocity;
    }

    private double getMaxAcceleration() {
        double effectiveAcceleration = maxAcceleration.get() * multiplier;
        aKitLog.record("EffectiveAcceleration", effectiveAcceleration);
        return effectiveAcceleration;
    }

    public void setTargetPosition(double targetValue, double currentValue, double currentVelocity) {
        // if the profile's constraints properties have changed, recompute the profile
        // there's maybe a better place to do this but this should be fine since setTarget will be called
        // over and over again
        multiplier = globalSafeSpeedsManager.getMultiplier();
        double maxVelocity = getMaxVelocity();
        double maxAcceleration = getMaxAcceleration();

        if (constraints.maxVelocity != maxVelocity || constraints.maxAcceleration != maxAcceleration) {
            constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
            profile = new TrapezoidProfile(constraints);
        }

        if(Math.abs(previousSetpoint - currentValue) > maxGap.get()){
            resetState(previousSetpoint - currentValue <= 0
                    ? currentValue - maxGap.get()
                    : currentValue + maxGap.get(), currentVelocity);
            return;
        }

        // if the target has changed, recompute the goal and current states
        if (goalState.position != targetValue) {
            // if the previousSentpoint we have is very recent, use it as the initial state to
            // avoid discontinuities in the profile
            if (XTimer.getFPGATimestampTime().minus(previousSetpointTime).in(Seconds) < 0.1) {
                initialState = new TrapezoidProfile.State(previousSetpoint, currentVelocity);
            } else {
                initialState = new TrapezoidProfile.State(currentValue, currentVelocity);
            }
            goalState = new TrapezoidProfile.State(targetValue, 0);
            profileStartTime.mut_replace(XTimer.getFPGATimestampTime().minus(Seconds.of(0.02)));
        }
    }

    // currently only doing position, but in theory this goal has a velocity associated with it too we could use
    public double getRecommendedPositionForTime() {
        var setpoint = profile.calculate(XTimer.getFPGATimestampTime().minus(profileStartTime).in(Seconds), initialState, goalState);
        this.previousSetpointTime.mut_replace(XTimer.getFPGATimestampTime());
        previousSetpoint = setpoint.position;
        return setpoint.position;
    }
    
}
