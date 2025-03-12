package org.tash.extensions.weather.avoid;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.HazardousWeather;

import java.time.ZonedDateTime;
import java.util.List;

/**
 * Interface for weather avoidance strategies
 */
public interface WeatherAvoidanceStrategy {
    /**
     * Find a path that avoids hazardous weather
     *
     * @param start   Start point
     * @param end     End point
     * @param time    Time of travel
     * @param hazards List of hazardous weather to avoid
     * @return List of waypoints forming a safe path (including start and end)
     */
    List<GeoCoordinate> findAvoidancePath(GeoCoordinate start, GeoCoordinate end,
                                          ZonedDateTime time, List<HazardousWeather> hazards);

    /**
     * Check if a direct path is safe
     *
     * @param start   Start point
     * @param end     End point
     * @param time    Time of travel
     * @param hazards List of hazardous weather to check
     * @return True if the direct path is safe
     */
    default boolean isDirectPathSafe(GeoCoordinate start, GeoCoordinate end,
                                     ZonedDateTime time, List<HazardousWeather> hazards) {
        // Check if path intersects any hazardous weather
        for (HazardousWeather hazard : hazards) {
            if (hazard instanceof WeatherCell) {
                WeatherCell cell = (WeatherCell) hazard;
                if (cell.intersectsPath(start, end, time)) {
                    return false;
                }
            } else if (hazard.affectsPoint(start, time) || hazard.affectsPoint(end, time)) {
                // For non-cell hazards, just check the endpoints
                return false;
            }
        }

        return true;
    }
}
