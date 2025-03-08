package competition.motion;

import edu.wpi.first.math.geometry.Translation2d;

public record CubicHermiteSplineParameters(Translation2d startPoint, Translation2d endPoint, Translation2d startControlVector, Translation2d endControlVector) {
    public CubicHermiteSplineParameters() {
        this(new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d());
    }
}
