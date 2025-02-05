package competition.motion;

import static edu.wpi.first.units.Units.Seconds;

import dagger.assisted.Assisted;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Time;
import xbot.common.controls.sensors.XTimer;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public class TrapezoidProfileManager {

    @AssistedFactory
    public abstract static class Factory {
        public abstract TrapezoidProfileManager create(
                @Assisted String name,
                @Assisted("defaultMaxVelocity") double defaultMaxVelocity,
                @Assisted("defaultMaxAcceleration") double defaultMaxAcceleration,
                @Assisted("initialPosition") double initialPosition);
    }

    final DoubleProperty maxVelocity;
    final DoubleProperty maxAcceleration;
    
    TrapezoidProfile profile;
    TrapezoidProfile.Constraints constraints;
    TrapezoidProfile.State initialState;
    TrapezoidProfile.State goalState;
    Time profileStartTime = Seconds.zero(); 

    @AssistedInject
    public TrapezoidProfileManager(
            @Assisted String name,
            PropertyFactory pf,
            @Assisted("defaultMaxVelocity") double defaultMaxVelocity,
            @Assisted("defaultMaxAcceleration") double defaultMaxAcceleration,
            @Assisted("initialPosition") double initialPosition) {
        pf.setPrefix(name);
        maxVelocity = pf.createPersistentProperty("maxVelocity", defaultMaxVelocity);
        maxAcceleration = pf.createPersistentProperty("maxAcceleration", defaultMaxAcceleration);
        constraints = new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
        profile = new TrapezoidProfile(constraints);
        // initialize states to current value
        initialState = new TrapezoidProfile.State(initialPosition, 0);
        goalState = new TrapezoidProfile.State(initialPosition, 0);
    }

    public void setTargetPosition(double targetValue, double currentValue, double currentVelocity) {
        // if the profile's constraints properties have changed, recompute the profile
        // there's maybe a better place to do this but this should be fine since setTarget will be called
        // over and over again
        if(constraints.maxVelocity != maxVelocity.get() || constraints.maxAcceleration != maxAcceleration.get()) {
            constraints = new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
            profile = new TrapezoidProfile(constraints);
        }

        // if the target has changed, recompute the goal and current states
        if(goalState.position != targetValue) {
            initialState = new TrapezoidProfile.State(currentValue, currentVelocity);
            goalState = new TrapezoidProfile.State(targetValue, 0);
            profileStartTime = XTimer.getFPGATimestampTime();
        }

    }

    // currently only doing position, but in theory this goal has a velocity associated with it too we could use
    public double getRecommendedPositionForTime() {
        var setpoint = profile.calculate(XTimer.getFPGATimestampTime().minus(profileStartTime).in(Seconds), initialState, goalState);
        return setpoint.position;
    }
    
}
