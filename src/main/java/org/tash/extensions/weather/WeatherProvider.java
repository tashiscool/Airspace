package org.tash.extensions.weather;

import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.List;

/**
 * Interface for all weather data providers
 */
public interface WeatherProvider {
    /**
     * Get all weather elements at a specific location and time
     *
     * @param location The geographic location
     * @param time     The time to query
     * @return List of weather elements at the location and time
     */
    List<WeatherElement> getWeatherAt(GeoCoordinate location, ZonedDateTime time);

    /**
     * Get wind at a specific location and time
     *
     * @param location The geographic location
     * @param time     The time to query
     * @return Wind vector at the location and time
     */
    WindComponent getWindAt(GeoCoordinate location, ZonedDateTime time);

    /**
     * Get all hazardous weather that affects a location at a specific time
     *
     * @param location The geographic location
     * @param time     The time to query
     * @return List of hazardous weather elements affecting the location
     */
    List<HazardousWeather> getHazardsAt(GeoCoordinate location, ZonedDateTime time);

    /**
     * Get all hazardous weather within a region at a specific time
     *
     * @param minLat Minimum latitude of the region (degrees)
     * @param minLon Minimum longitude of the region (degrees)
     * @param maxLat Maximum latitude of the region (degrees)
     * @param maxLon Maximum longitude of the region (degrees)
     * @param minAlt Minimum altitude of the region (feet)
     * @param maxAlt Maximum altitude of the region (feet)
     * @param time   The time to query
     * @return List of hazardous weather elements in the region
     */
    List<HazardousWeather> getHazardsInRegion(
            double minLat, double minLon, double maxLat, double maxLon,
            double minAlt, double maxAlt, ZonedDateTime time);

    /**
     * Check if a location is safe (free from severe hazards) at a specific time
     *
     * @param location The geographic location
     * @param time     The time to query
     * @return True if the location is safe
     */
    boolean isSafe(GeoCoordinate location, ZonedDateTime time);
}
