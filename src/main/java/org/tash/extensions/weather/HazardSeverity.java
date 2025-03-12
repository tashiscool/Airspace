package org.tash.extensions.weather;

/**
 * Severity levels for hazardous weather
 */
public enum HazardSeverity {
    LIGHT(0.5),
    MODERATE(1.0),
    SEVERE(5.0),
    EXTREME(100.0); // Effectively impassable

    private final double costFactor;

    HazardSeverity(double costFactor) {
        this.costFactor = costFactor;
    }

    public double getCostFactor() {
        return costFactor;
    }
}
