package org.tash.extensions.weather.wind;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.WindComponent;

import java.time.ZonedDateTime;

/**
 * Interface for wind field models that provide wind data at any location and time
 */
public interface WindFieldModel {
    /**
     * Get the wind at a specific location and time
     * @param location The geographic location
     * @param time The time to query
     * @return The wind component at the location and time
     */
    WindComponent getWindAt(GeoCoordinate location, ZonedDateTime time);
    
    /**
     * Get the wind field validity time
     * @return The validity time of the wind field data
     */
    ZonedDateTime getValidityTime();
    
    /**
     * Get the wind field ID
     * @return The wind field model identifier
     */
    String getId();
}

