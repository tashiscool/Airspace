package org.tash.extensions.weather.avoid;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.geodetic.GeodeticCalculator;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * A circular weather cell (e.g., thunderstorm cell, thermal)
 */
@Data
public class CircularWeatherCell extends WeatherCell {
    private final GeoCoordinate center;
    private final double radius; // nautical miles

    @Builder
    public CircularWeatherCell(
            String id,
            WeatherElementType type,
            HazardSeverity severity,
            ZonedDateTime startTime,
            ZonedDateTime endTime,
            double minAltitude,
            double maxAltitude,
            GeoCoordinate center,
            double radius) {
        super(id, type, severity, startTime, endTime, minAltitude, maxAltitude);
        this.center = center;
        this.radius = radius;
    }

    @Override
    protected boolean containsHorizontally(GeoCoordinate point) {
        return center.distanceTo(point) <= radius;
    }

    @Override
    public List<GeoCoordinate> getBoundaryPoints() {
        // Generate points around the circle
        int numPoints = 36; // one point every 10 degrees
        List<GeoCoordinate> points = new ArrayList<>(numPoints);

        for (int i = 0; i < numPoints; i++) {
            double angle = 360.0 * i / numPoints;
            points.add(center.destinationPoint(radius, angle));
        }

        return points;
    }

    @Override
    protected boolean doesPathIntersectBoundary(GeoCoordinate start, GeoCoordinate end) {
        // Calculate cross-track distance from path to center
        double crossTrack = GeodeticCalculator.crossTrackDistance(start, end, center);

        // Compare to radius
        return Math.abs(crossTrack) <= radius;
    }
}
