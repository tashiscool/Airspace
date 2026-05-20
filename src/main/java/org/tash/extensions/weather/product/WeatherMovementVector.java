package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.Duration;

@Data
@Builder
public class WeatherMovementVector {
    private final double speedNauticalMilesPerHour;
    private final double bearingDegrees;
    private final double verticalFeetPerMinute;

    public GeoCoordinate project(GeoCoordinate point, Duration elapsed) {
        if (point == null || elapsed == null) {
            return point;
        }
        double hours = elapsed.toMillis() / 3_600_000.0;
        double distance = speedNauticalMilesPerHour * hours;
        GeoCoordinate moved = point.destinationPoint(distance, bearingDegrees);
        return moved.withAltitude(point.getAltitude() + verticalFeetPerMinute * elapsed.toMinutes());
    }
}
