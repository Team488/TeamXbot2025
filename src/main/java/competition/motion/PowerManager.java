package competition.motion;

import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class PowerManager {

    // RANGE: 0-1
    private final DoubleProperty multiplier;

    @Inject
    public PowerManager(PropertyFactory pf) {
        pf.setPrefix("PowerManager/");
        this.multiplier = pf.createPersistentProperty(pf.getPrefix(), 1);
    }

    /**
     * @return the multiplier constrained 0 to 1
     */
    public double getMultiplier() {
        return Math.max(0, Math.min(1, multiplier.get()));
    }
}
