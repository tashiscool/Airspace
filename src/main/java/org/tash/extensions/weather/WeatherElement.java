package org.tash.extensions.weather;

import java.time.ZonedDateTime;

/**
 * Base interface for all weather phenomena
 */
public interface WeatherElement {
    /**
     * Get the identifier for this weather element
     * @return The unique identifier
     */
    String getId();
    
    /**
     * Get the type of weather element
     * @return The weather element type
     */
    WeatherElementType getType();
    
    /**
     * Get the validity time of this weather element
     * @return The validity time
     */
    ZonedDateTime getValidityTime();
    
    /**
     * Check if the weather element is valid at a specific time
     * @param time The time to check
     * @return True if the weather element is valid at the specified time
     */
    boolean isValidAt(ZonedDateTime time);
}

