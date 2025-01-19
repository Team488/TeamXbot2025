package competition.simulation;

import edu.wpi.first.math.geometry.Pose2d;

public interface BaseSimulator {
    public void update();

    public void resetPosition(Pose2d pose);
}
