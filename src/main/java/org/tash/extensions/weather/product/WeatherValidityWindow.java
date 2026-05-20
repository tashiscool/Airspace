package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

import java.time.Duration;
import java.time.ZonedDateTime;

@Data
@Builder
public class WeatherValidityWindow {
    private final ZonedDateTime validStart;
    private final ZonedDateTime validEnd;

    public boolean contains(ZonedDateTime time) {
        if (time == null) {
            return false;
        }
        if (validStart != null && time.isBefore(validStart)) {
            return false;
        }
        return validEnd == null || !time.isAfter(validEnd);
    }

    public boolean isExpiredAt(ZonedDateTime time) {
        return time != null && validEnd != null && time.isAfter(validEnd);
    }

    public Duration duration() {
        if (validStart == null || validEnd == null) {
            return Duration.ZERO;
        }
        return Duration.between(validStart, validEnd);
    }
}
