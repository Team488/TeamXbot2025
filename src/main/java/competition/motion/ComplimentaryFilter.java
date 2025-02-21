package competition.motion;

import xbot.common.advantage.AKitLogger;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import java.util.Optional;

public class ComplimentaryFilter {

    final DoubleProperty biasUpdateFactor;
    final DoubleProperty preciseSensorWeight;
    final DoubleProperty accurateSensorUpperLimitProperty;
    final String owningSystemPrefix;
    final AKitLogger aKitLog;

    double biasEstimate = 0;

    public ComplimentaryFilter(PropertyFactory pf,
                               String owningSystemPrefix,
                               boolean accurateSensorHasUpperLimit,
                               double upperLimitDefault) {
        pf.setPrefix(owningSystemPrefix + "ComplimentaryFilter");
        this.owningSystemPrefix = owningSystemPrefix;
        aKitLog = new AKitLogger(owningSystemPrefix + "ComplimentaryFilter");

        biasUpdateFactor = pf.createPersistentProperty("Bias Update Factor", 0.01);
        preciseSensorWeight = pf.createPersistentProperty("Precise Sensor Weight", 0.98);

        if (accurateSensorHasUpperLimit) {
            accurateSensorUpperLimitProperty =
                    pf.createPersistentProperty("Accurate Sensor Upper Limit", upperLimitDefault);
        } else {
            accurateSensorUpperLimitProperty = null;
        }
    }

    public ComplimentaryFilter(PropertyFactory pf, String owningSystemPrefix) {
        this(pf, owningSystemPrefix, false, Double.MAX_VALUE);
    }

    public void setAccurateSensorUpperLimit(double limit) {
        if (accurateSensorUpperLimitProperty != null) {
            accurateSensorUpperLimitProperty.set(limit);
        }
    }

    private double getAccurateSensorUpperLimit() {
        if (accurateSensorUpperLimitProperty != null) {
            return accurateSensorUpperLimitProperty.get();
        } else {
            return Double.MAX_VALUE;
        }
    }

    public double calculateFilteredValue(double accurateSensorReading, double preciseSensorReading) {

        double biasUpdateFactor = this.biasUpdateFactor.get();
        double accurateSensorUpperLimit = getAccurateSensorUpperLimit();

        if (accurateSensorReading < accurateSensorUpperLimit) {
            // We can trust the accurate sensor here; update the bias estimate.
            biasEstimate = (1 - biasUpdateFactor) * biasEstimate + biasUpdateFactor
                    * (preciseSensorReading - accurateSensorReading);
        }

        // Correct the precise measurement with our estimated bias
        double correctedPrecise = preciseSensorReading - biasEstimate;

        double fused = 0;
        if (accurateSensorReading > accurateSensorUpperLimit) {
            // If the accurate sensor is unreliable, just use the precise sensor.
            fused = correctedPrecise;
        } else {
            // If the accurate sensor is reliable, use a weighted average of the two sensors.
            double preciseSensorWeight = this.preciseSensorWeight.get();
            fused = preciseSensorWeight * correctedPrecise
                    + (1 - preciseSensorWeight) * accurateSensorReading;
        }

        aKitLog.setLogLevel(AKitLogger.LogLevel.DEBUG);
        aKitLog.record("Bias Estimate", biasEstimate);
        aKitLog.record("Corrected Precise", correctedPrecise);
        aKitLog.setLogLevel(AKitLogger.LogLevel.INFO);

        return fused;
    }
}
