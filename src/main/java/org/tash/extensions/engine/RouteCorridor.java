package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class RouteCorridor {
    @Builder.Default
    private final List<GeoCoordinate> points = new ArrayList<>();
    @Builder.Default
    private final double bufferNauticalMiles = 25.0;

    public List<GeoCoordinate> getPoints() {
        return Collections.unmodifiableList(points);
    }

    public double minLatitude() {
        return points.stream().mapToDouble(GeoCoordinate::getLatitude).min().orElse(-90.0) - degrees(bufferNauticalMiles);
    }

    public double maxLatitude() {
        return points.stream().mapToDouble(GeoCoordinate::getLatitude).max().orElse(90.0) + degrees(bufferNauticalMiles);
    }

    public double minLongitude() {
        return points.stream().mapToDouble(GeoCoordinate::getLongitude).min().orElse(-180.0) - degrees(bufferNauticalMiles);
    }

    public double maxLongitude() {
        return points.stream().mapToDouble(GeoCoordinate::getLongitude).max().orElse(180.0) + degrees(bufferNauticalMiles);
    }

    private double degrees(double nauticalMiles) {
        return nauticalMiles / 60.0;
    }
}
