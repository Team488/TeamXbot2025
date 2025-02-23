package competition.subsystems.oracle;

import xbot.common.math.XYPair;

public record FieldOrientedDriveData(XYPair fieldOrientedDriveIntent,
                                     double rotationIntent) {
    public FieldOrientedDriveData() {
        this(new XYPair(0, 0), 0);
    }
}
