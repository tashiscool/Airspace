package org.tash.time;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * Motion model for helical trajectory
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class HelicalMotionModel implements MotionModel {
    private GeoCoordinate center;
    private double radius;
    private double startAngle;
    private double endAngle;
    private double startAltitude;
    private double endAltitude;
    private boolean clockwise;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;

    @Override
    public GeoCoordinate getPositionAt(ZonedDateTime time) {
        // Check if time is outside the segment timeframe
        if (time.isBefore(startTime)) {
            return getPositionAtFraction(0);
        } else if (time.isAfter(endTime)) {
            return getPositionAtFraction(1);
        }

        // Calculate the time fraction (0 to 1)
        double totalMillis = java.time.Duration.between(startTime, endTime).toMillis();
        double elapsedMillis = java.time.Duration.between(startTime, time).toMillis();
        double fraction = totalMillis > 0 ? elapsedMillis / totalMillis : 0;

        return getPositionAtFraction(fraction);
    }

    /**
     * Get position at a specific fraction along the helix
     */
    public GeoCoordinate getPositionAtFraction(double fraction) {
        // Clamp fraction between 0 and 1
        fraction = Math.max(0, Math.min(1, fraction));

        // Calculate angle at fraction
        double angleRange = getAngleRange();
        double angle = startAngle + (clockwise ? -1 : 1) * fraction * angleRange;

        // Calculate altitude at fraction (linear interpolation)
        double altitude = startAltitude + fraction * (endAltitude - startAltitude);

        // Convert angle to radians
        double angleRad = Math.toRadians(angle);

        // Calculate position offset from center
        double lat = center.getLatitude();
        double lon = center.getLongitude();

        // Convert radius from nautical miles to appropriate deltas
        double latDelta = radius * Math.sin(angleRad) / 60.0;  // 1 minute of latitude = 1 NM
        double lonDelta = radius * Math.cos(angleRad) / (60.0 * Math.cos(Math.toRadians(lat)));

        return GeoCoordinate.builder()
                .latitude(lat + latDelta)
                .longitude(lon + lonDelta)
                .altitude(altitude)
                .build();
    }

    @Override
    public TimeInterval getTimeInterval() {
        return new TimeInterval(startTime, endTime);
    }

    /**
     * Get the angular range of the helix in degrees
     */
    private double getAngleRange() {
        double angleDiff = endAngle - startAngle;

        // Normalize to handle the -180 to +180 range properly
        while (angleDiff > 180) angleDiff -= 360;
        while (angleDiff < -180) angleDiff += 360;

        // Apply direction
        if (clockwise && angleDiff > 0) angleDiff -= 360;
        if (!clockwise && angleDiff < 0) angleDiff += 360;

        return Math.abs(angleDiff);
    }

    /**
     * Get a list of discrete points along the helix path
     *
     * @param numPoints Number of points to generate
     * @return List of points along the helix
     */
    public List<GeoCoordinate> getHelixPoints(int numPoints) {
        List<GeoCoordinate> points = new ArrayList<>();

        for (int i = 0; i < numPoints; i++) {
            double fraction = (double) i / (numPoints - 1);
            points.add(getPositionAtFraction(fraction));
        }

        return points;
    }

    /**
     * Calculate the 3D representation of this helix as a set of vectors
     *
     * @param numPoints Number of points to use
     * @return List of 3D vectors representing the helix
     */
    public List<RealVector> getHelixVectors(int numPoints) {
        List<RealVector> vectors = new ArrayList<>();

        // Get points along the helix
        List<GeoCoordinate> points = getHelixPoints(numPoints);

        // Convert to ECEF coordinates for better 3D representation
        for (GeoCoordinate point : points) {
            // Use Apache Commons Math RealVector for 3D operations
            double[] coords = new double[3];

            // Convert latitude/longitude to a local Cartesian system relative to center
            double lat = Math.toRadians(point.getLatitude());
            double lon = Math.toRadians(point.getLongitude());
            double centerLat = Math.toRadians(center.getLatitude());
            double centerLon = Math.toRadians(center.getLongitude());

            // Simple approximation using equirectangular projection
            double x = (lon - centerLon) * Math.cos((lat + centerLat) / 2);
            double y = (lat - centerLat);

            // Scale by Earth radius to get distances in nautical miles
            double earthRadius = 3440.065; // in nautical miles
            coords[0] = x * earthRadius;
            coords[1] = y * earthRadius;
            coords[2] = point.getAltitude() / 6076.12; // convert feet to nautical miles

            vectors.add(new ArrayRealVector(coords));
        }

        return vectors;
    }
}
