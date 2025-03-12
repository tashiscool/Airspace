package org.tash.extensions.weather.wind;

import org.tash.extensions.weather.WeatherElement;
import org.tash.extensions.weather.WeatherElementType;

import java.time.ZonedDateTime;

/**
 * Abstract base class for wind field implementations
 */
public abstract class AbstractWindField implements WindFieldModel, WeatherElement {
    protected final String id;
    protected final ZonedDateTime validityTime;

    public AbstractWindField(String id, ZonedDateTime validityTime) {
        this.id = id;
        this.validityTime = validityTime;
    }

    @Override
    public String getId() {
        return id;
    }

    @Override
    public WeatherElementType getType() {
        return WeatherElementType.WIND;
    }

    @Override
    public ZonedDateTime getValidityTime() {
        return validityTime;
    }

    @Override
    public boolean isValidAt(ZonedDateTime time) {
        // Default implementation considers the model valid for +/- 1 hour from validity time
        long diffMinutes = Math.abs(java.time.Duration.between(validityTime, time).toMinutes());
        return diffMinutes <= 60;
    }
}
