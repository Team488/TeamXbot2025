package competition.motion;

    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.math.trajectory.TrapezoidProfile;
    import xbot.common.controls.sensors.XTimer;

    import javax.inject.Inject;
    import java.util.ArrayList;
    import java.util.List;

    public class HermiteTrajectory {

        private List<CubicHermiteSpline> splines;
        private TrapezoidProfile trapezoid;
        private double startTimeInSeconds = 0;
        private double elapsedSeconds = 0;
        private double frozenOffsetSeconds = 0;
        private final double maxGap = 0.25;
        private TrapezoidProfile.State initialState;
        private TrapezoidProfile.State finalState;

        // Store cumulative arc lengths for each spline
        private double[] cumulativeArcLengths;

        @Inject
        public HermiteTrajectory() {
            trapezoid = new TrapezoidProfile(new TrapezoidProfile.Constraints(4, 3));
            splines = new ArrayList<>();
        }

        public void setSplines(List<CubicHermiteSpline> splines) {
            this.splines = splines;
        }

        public void setSpline(CubicHermiteSpline spline) {
            this.splines = new ArrayList<>();
            this.splines.add(spline);
        }

        public void initialize(double currentVelocityMetersPerSecond, double finalVelocityMetersPerSecond) {
            // Initialize arc length tables for all splines
            for (CubicHermiteSpline spline : splines) {
                spline.initializeArcLengthTable();
            }

            // Calculate cumulative arc lengths
            calculateCumulativeArcLengths();

            // Total arc length is last value in cumulativeArcLengths
            double arcLengthMeters = cumulativeArcLengths[splines.size() - 1];

            startTimeInSeconds = XTimer.getFPGATimestamp();
            initialState = new TrapezoidProfile.State(0, currentVelocityMetersPerSecond);
            finalState = new TrapezoidProfile.State(arcLengthMeters, finalVelocityMetersPerSecond);
            frozenOffsetSeconds = 0;
            elapsedSeconds = 0;
        }

        private void calculateCumulativeArcLengths() {
            cumulativeArcLengths = new double[splines.size()];
            double totalLength = 0;

            for (int i = 0; i < splines.size(); i++) {
                double splineLength = splines.get(i).estimateArcLength();
                totalLength += splineLength;
                cumulativeArcLengths[i] = totalLength;
            }
        }

        public HermiteTrajectoryAdvice advise(Translation2d currentPosition) {
            double elapsedSecondsCandidate = XTimer.getFPGATimestamp() - startTimeInSeconds - frozenOffsetSeconds;
            var candidateAdvice = getAdviceForTime(elapsedSecondsCandidate, false);

            if (Math.abs(candidateAdvice.position().getDistance(currentPosition)) > maxGap) {
                candidateAdvice = getAdviceForTime(elapsedSeconds, true);
                frozenOffsetSeconds = XTimer.getFPGATimestamp() - startTimeInSeconds - elapsedSeconds;
            } else {
                elapsedSeconds = elapsedSecondsCandidate;
            }

            return candidateAdvice;
        }

        private HermiteTrajectoryAdvice getAdviceForTime(double elapsedSeconds, boolean timeFrozen) {
            var trapezoidAdvice = trapezoid.calculate(elapsedSeconds, initialState, finalState);
            double totalDistance = trapezoidAdvice.position;

            // Find which spline contains this distance
            int splineIndex = 0;
            double previousCumulativeLength = 0;

            for (int i = 0; i < cumulativeArcLengths.length; i++) {
                if (totalDistance <= cumulativeArcLengths[i]) {
                    splineIndex = i;
                    break;
                }
                previousCumulativeLength = cumulativeArcLengths[i];
            }

            // Calculate local distance within the selected spline
            double localDistance = totalDistance - previousCumulativeLength;

            // Get the correct spline and evaluate it
            CubicHermiteSpline currentSpline = splines.get(splineIndex);
            double splineLength = (splineIndex == 0) ? cumulativeArcLengths[0] :
                                  cumulativeArcLengths[splineIndex] - cumulativeArcLengths[splineIndex-1];
            double lerp = currentSpline.getParameterFromDistance(localDistance);

            var position = currentSpline.evaluate(lerp);
            var direction = currentSpline.derivative(lerp);
            var velocity = new Translation2d(trapezoidAdvice.velocity, direction.getAngle());

            return new HermiteTrajectoryAdvice(position, velocity, timeFrozen, trapezoid.timeLeftUntil(finalState.position));
        }
    }