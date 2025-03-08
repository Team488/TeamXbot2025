package competition.motion;

import edu.wpi.first.math.geometry.Translation2d;

public record HermiteTrajectoryAdvice(Translation2d position, Translation2d velocity, boolean timeFrozen, double timeRemaining, double distanceTravelled) {
}
