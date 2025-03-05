package competition.motion;

    import edu.wpi.first.math.geometry.Translation2d;

    /**
     * Represents a Cubic Hermite spline with methods to set points, control vectors,
     * and estimate arc length using Gauss-Legendre quadrature.
     */
    public class CubicHermiteSpline {
        // Points
        private Translation2d startPoint;
        private Translation2d endPoint;

        // Control vectors
        private Translation2d startControlVector;
        private Translation2d endControlVector;

        // Lookup table for arc length parameterization
        private boolean arcLengthTableInitialized = false;
        private double[] arcLengthTable;
        private int lookupTableSize = 50;  // Balance between accuracy and performance

        /**
         * Creates a new Cubic Hermite spline with all points and vectors initialized to zero.
         */
        public CubicHermiteSpline() {
            startPoint = new Translation2d(0.0, 0.0);
            endPoint = new Translation2d(0.0, 0.0);
            startControlVector = new Translation2d(0.0, 0.0);
            endControlVector = new Translation2d(0.0, 0.0);
        }

        public CubicHermiteSpline(CubicHermiteSplineParameters params) {
            setParameters(params);
        }

        /**
         * Sets the starting point of the spline.
         */
        public void setStartPoint(Translation2d point) {
            this.startPoint = point;
        }

        /**
         * Sets the end point of the spline.
         */
        public void setEndPoint(Translation2d point) {
            this.endPoint = point;
        }

        /**
         * Sets the starting control vector using magnitude and direction.
         */
        public void setStartControlVector(double magnitude, double directionRadians) {
            double x = magnitude * Math.cos(directionRadians);
            double y = magnitude * Math.sin(directionRadians);
            this.startControlVector = new Translation2d(x, y);
        }

        /**
         * Sets the end control vector using magnitude and direction.
         */
        public void setEndControlVector(double magnitude, double directionRadians) {
            double x = magnitude * Math.cos(directionRadians);
            double y = magnitude * Math.sin(directionRadians);
            this.endControlVector = new Translation2d(x, y);
        }

        /**
         * Sets the starting control vector using a Translation2d.
         */
        public void setStartControlVector(Translation2d vector) {
            this.startControlVector = vector;
        }

        /**
         * Sets the end control vector using a Translation2d.
         */
        public void setEndControlVector(Translation2d vector) {
            this.endControlVector = vector;
        }

        public void setParameters(CubicHermiteSplineParameters params) {
            setStartPoint(params.startPoint());
            setEndPoint(params.endPoint());
            setStartControlVector(params.startControlVector());
            setEndControlVector(params.endControlVector());
        }

        /**
         * Evaluates the spline at parameter t.
         */
        public Translation2d evaluate(double t) {
            // Hermite basis functions
            double h00 = 2*t*t*t - 3*t*t + 1;
            double h10 = t*t*t - 2*t*t + t;
            double h01 = -2*t*t*t + 3*t*t;
            double h11 = t*t*t - t*t;

            double x = h00 * startPoint.getX() + h10 * startControlVector.getX() +
                       h01 * endPoint.getX() + h11 * endControlVector.getX();
            double y = h00 * startPoint.getY() + h10 * startControlVector.getY() +
                       h01 * endPoint.getY() + h11 * endControlVector.getY();

            return new Translation2d(x, y);
        }

        /**
         * Evaluates the x-coordinate of the spline at parameter t.
         */
        public double evaluateX(double t) {
            // Hermite basis functions
            double h00 = 2*t*t*t - 3*t*t + 1;
            double h10 = t*t*t - 2*t*t + t;
            double h01 = -2*t*t*t + 3*t*t;
            double h11 = t*t*t - t*t;

            return h00 * startPoint.getX() + h10 * startControlVector.getX() +
                   h01 * endPoint.getX() + h11 * endControlVector.getX();
        }

        /**
         * Evaluates the y-coordinate of the spline at parameter t.
         */
        public double evaluateY(double t) {
            // Hermite basis functions
            double h00 = 2*t*t*t - 3*t*t + 1;
            double h10 = t*t*t - 2*t*t + t;
            double h01 = -2*t*t*t + 3*t*t;
            double h11 = t*t*t - t*t;

            return h00 * startPoint.getY() + h10 * startControlVector.getY() +
                   h01 * endPoint.getY() + h11 * endControlVector.getY();
        }

        /**
         * Computes the derivative at parameter t.
         */
        public Translation2d derivative(double t) {
            double dh00 = 6*t*t - 6*t;
            double dh10 = 3*t*t - 4*t + 1;
            double dh01 = -6*t*t + 6*t;
            double dh11 = 3*t*t - 2*t;

            double dx = dh00 * startPoint.getX() + dh10 * startControlVector.getX() +
                       dh01 * endPoint.getX() + dh11 * endControlVector.getX();
            double dy = dh00 * startPoint.getY() + dh10 * startControlVector.getY() +
                       dh01 * endPoint.getY() + dh11 * endControlVector.getY();

            return new Translation2d(dx, dy);
        }

        /**
         * Computes the derivative of x with respect to t.
         */
        public double derivativeX(double t) {
            double dh00 = 6*t*t - 6*t;
            double dh10 = 3*t*t - 4*t + 1;
            double dh01 = -6*t*t + 6*t;
            double dh11 = 3*t*t - 2*t;

            return dh00 * startPoint.getX() + dh10 * startControlVector.getX() +
                   dh01 * endPoint.getX() + dh11 * endControlVector.getX();
        }

        /**
         * Computes the derivative of y with respect to t.
         */
        public double derivativeY(double t) {
            double dh00 = 6*t*t - 6*t;
            double dh10 = 3*t*t - 4*t + 1;
            double dh01 = -6*t*t + 6*t;
            double dh11 = 3*t*t - 2*t;

            return dh00 * startPoint.getY() + dh10 * startControlVector.getY() +
                   dh01 * endPoint.getY() + dh11 * endControlVector.getY();
        }

        /**
         * Estimates the arc length of the spline using 5-point Gauss-Legendre quadrature.
         */
        public double estimateArcLength() {
            // 5-point Gauss-Legendre quadrature points and weights
            double[] points = {
                -0.9061798459,
                -0.5384693101,
                0.0,
                0.5384693101,
                0.9061798459
            };

            double[] weights = {
                0.2369268850,
                0.4786286705,
                0.5688888889,
                0.4786286705,
                0.2369268850
            };

            double sum = 0.0;

            // Scale from [-1, 1] to [0, 1]
            for (int i = 0; i < points.length; i++) {
                double t = (points[i] + 1) / 2.0;
                double dx = derivativeX(t);
                double dy = derivativeY(t);
                double integrand = Math.sqrt(dx * dx + dy * dy);
                sum += weights[i] * integrand;
            }

            // Scale factor for interval change
            return sum * 0.5;
        }

        /**
         * Returns the parameter t corresponding to a given distance along the spline.
         * Uses a precomputed lookup table for efficiency.
         *
         * @param distance The distance along the spline from the start point
         * @return The parameter t (between 0 and 1) corresponding to that distance
         */
        public double getParameterFromDistance(double distance) {
            // Handle edge cases
            if (distance <= 0) {
                return 0.0;
            }

            // Initialize table if needed
            if (!arcLengthTableInitialized) {
                initializeArcLengthTable();
            }

            double totalLength = arcLengthTable[lookupTableSize - 1];
            if (distance >= totalLength) {
                return 1.0;
            }

            // Binary search to find the segment containing our target distance
            int low = 0;
            int high = lookupTableSize - 1;

            while (low < high - 1) {
                int mid = (low + high) / 2;
                if (arcLengthTable[mid] < distance) {
                    low = mid;
                } else {
                    high = mid;
                }
            }

            // Linear interpolation between table entries
            double t0 = (double)low / (lookupTableSize - 1);
            double t1 = (double)high / (lookupTableSize - 1);
            double s0 = arcLengthTable[low];
            double s1 = arcLengthTable[high];

            return t0 + (t1 - t0) * (distance - s0) / (s1 - s0);
        }

        /**
         * Initialize the arc length lookup table using Gauss-Legendre quadrature
         */
        public void initializeArcLengthTable() {
            arcLengthTable = new double[lookupTableSize];
            arcLengthTable[0] = 0;

            // 3-point Gauss-Legendre quadrature for efficiency
            double[] points = {-0.7745966692, 0.0, 0.7745966692};
            double[] weights = {0.5555555556, 0.8888888889, 0.5555555556};

            // For each segment in our lookup table
            for (int i = 1; i < lookupTableSize; i++) {
                double t = (double)i / (lookupTableSize - 1);
                double tPrev = (double)(i-1) / (lookupTableSize - 1);
                double segmentLength = 0;

                // Apply Gauss-Legendre for this segment
                for (int j = 0; j < points.length; j++) {
                    // Map from [-1, 1] to [tPrev, t]
                    double tau = ((t - tPrev) * points[j] + t + tPrev) / 2;

                    double dx = derivativeX(tau);
                    double dy = derivativeY(tau);
                    double speed = Math.sqrt(dx * dx + dy * dy);

                    segmentLength += weights[j] * speed;
                }

                // Scale for the interval and add to total
                segmentLength *= (t - tPrev) / 2;
                arcLengthTable[i] = arcLengthTable[i-1] + segmentLength;
            }

            arcLengthTableInitialized = true;
        }

        // Getters
        public Translation2d getStartPoint() { return startPoint; }
        public Translation2d getEndPoint() { return endPoint; }
        public Translation2d getStartControlVector() { return startControlVector; }
        public Translation2d getEndControlVector() { return endControlVector; }

        // Compatibility methods for legacy code
        public double getStartX() { return startPoint.getX(); }
        public double getStartY() { return startPoint.getY(); }
        public double getEndX() { return endPoint.getX(); }
        public double getEndY() { return endPoint.getY(); }
        public double getStartControlVectorX() { return startControlVector.getX(); }
        public double getStartControlVectorY() { return startControlVector.getY(); }
        public double getEndControlVectorX() { return endControlVector.getX(); }
        public double getEndControlVectorY() { return endControlVector.getY(); }
    }