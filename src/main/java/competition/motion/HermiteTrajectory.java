package competition.motion;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import xbot.common.controls.sensors.XTimer;

import javax.inject.Inject;

public class HermiteTrajectory {

    CubicHermiteSpline spline;
    TrapezoidProfile trapezoid;
    double arcLengthMeters;
    double startTimeInSeconds = 0;
    double elapsedSeconds = 0;
    double frozenOffsetSeconds = 0;
    double maxGap = 0.25;
    private TrapezoidProfile.State initialState;
    private TrapezoidProfile.State finalState;

    @Inject
    public HermiteTrajectory() {
        trapezoid = new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 1));
    }

    public void setSpline(CubicHermiteSpline spline) {
        this.spline = spline;
    }

    public void initialize(double currentVelocityMetersPerSecond, double finalVelocityMetersPerSecond) {
        arcLengthMeters = spline.estimateArcLength();
        startTimeInSeconds = XTimer.getFPGATimestamp();
        spline.initializeArcLengthTable();
        initialState = new TrapezoidProfile.State(0, currentVelocityMetersPerSecond);
        finalState = new TrapezoidProfile.State(arcLengthMeters, finalVelocityMetersPerSecond);
        frozenOffsetSeconds = 0;
        elapsedSeconds = 0;
    }

    public HermiteTrajectoryAdvice advise(Translation2d currentPosition) {

        double elapsedSecondsCandidate = XTimer.getFPGATimestamp() - startTimeInSeconds - frozenOffsetSeconds;
        // check to see how far away we are from our projected point
        var candidateAdvice = getAdviceForTime(elapsedSecondsCandidate, false);

        // Now do a quick check - if the positions are too far apart, fall back on the last time candidate.
        if (Math.abs(candidateAdvice.position().getDistance(currentPosition)) > maxGap) {
            // We need to freeze here until the robot catches up
            candidateAdvice = getAdviceForTime(elapsedSeconds, true);
            frozenOffsetSeconds = XTimer.getFPGATimestamp() - startTimeInSeconds - elapsedSeconds;
        } else {
            elapsedSeconds = elapsedSecondsCandidate;
        }

        return candidateAdvice;
    }

    private HermiteTrajectoryAdvice getAdviceForTime(double elapsedSeconds, boolean timeFrozen) {
        var trapezoidAdvice = trapezoid.calculate(elapsedSeconds, initialState, finalState);
        double lerp = spline.getParameterFromDistance(trapezoidAdvice.position);
        var position = spline.evaluate(lerp);
        var direction = spline.derivative(lerp);
        var velocity = new Translation2d(trapezoidAdvice.velocity, direction.getAngle());
        return new HermiteTrajectoryAdvice(position, velocity, timeFrozen);
    }
}
