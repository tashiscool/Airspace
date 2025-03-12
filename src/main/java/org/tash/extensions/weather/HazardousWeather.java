package org.tash.extensions.weather;

import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

/**
 * Base interface for all hazardous weather elements that should be avoided
 */
public interface HazardousWeather extends WeatherElement {
    /**
     * Get the severity level of the hazard
     *
     * @return The severity level
     */
    HazardSeverity getSeverity();

    /**
     * Check if a point is affected by this hazardous weather
     *
     * @param point The point to check
     * @param time  The time to check
     * @return True if the point is affected by this hazard
     */

    boolean affectsPoint(GeoCoordinate point, ZonedDateTime time);

    /**
     * Calculate the cost factor for passing through this hazard
     * Higher values indicate more severe penalties
     *
     * @return Cost factor multiplier (0.0 = no penalty, 1.0+ = increasing penalty)
     */
    double getCostFactor();
}
