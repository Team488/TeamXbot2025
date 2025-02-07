package competition.subsystems.drive;

import javax.inject.Inject;

import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.math.PIDManager;

public class DrivePowerCalculator {

    public static Translation2d getPowerToAchieveFieldPosition(
            Translation2d current, Translation2d target, PIDManager pid) {
        var goalVector = target.minus(current);
        // This essentially says "our goal is X distance away from where we are now.
        // Assume we are at zero. How much power should we use to go X distance?"
        double drivePower = pid.calculate(goalVector.getNorm(), 0);
        return new Translation2d(drivePower, goalVector.getAngle());
    }
}
