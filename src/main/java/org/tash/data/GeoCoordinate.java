package org.tash.data;

import lombok.*;
import org.tash.extensions.geodetic.ECEFCoordinate;
import org.tash.extensions.geodetic.ENUCoordinate;
import org.tash.extensions.geodetic.EarthModel;
import org.tash.extensions.geodetic.GeodeticCalculator;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

/**
 * Enhanced GeoCoordinate class with geodetic calculation methods
 * Note: This is an updated version of the original GeoCoordinate class
 * with additional methods for geodetic calculations
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class GeoCoordinate {
    /** Latitude in degrees */
    private double latitude;

    /** Longitude in degrees */
    private double longitude;

    /** Altitude in feet above WGS84 ellipsoid */
    private double altitude;

    /**
     * Convert to ECEF coordinates
     * @return ECEF coordinate in meters
     */
    public ECEFCoordinate toECEF() {
        return ECEFCoordinate.fromGeodetic(this);
    }

    /**
     * Convert to ENU coordinates relative to a reference point
     * @param reference The reference point (origin of ENU)
     * @return ENU coordinate in meters
     */
    public ENUCoordinate toENU(GeoCoordinate reference) {
        return ENUCoordinate.fromGeodetic(this, reference);
    }

    /**
     * Calculate distance to another point using Vincenty's formula
     * @param other Other geographic coordinate
     * @return Distance in nautical miles
     */
    public double distanceTo(GeoCoordinate other) {
        return GeodeticCalculator.vincentyDistance(this, other);
    }

    /**
     * Calculate the initial bearing to another point
     * @param other Other geographic coordinate
     * @return Bearing in degrees (0-360)
     */
    public double initialBearingTo(GeoCoordinate other) {
        return GeodeticCalculator.initialBearing(this, other);
    }

    /**
     * Calculate the final bearing to another point
     * @param other Other geographic coordinate
     * @return Bearing in degrees (0-360)
     */
    public double finalBearingTo(GeoCoordinate other) {
        return GeodeticCalculator.finalBearing(this, other);
    }

    /**
     * Find the midpoint between this coordinate and another on a great circle path
     * @param other Other geographic coordinate
     * @return Midpoint coordinate
     */
    public GeoCoordinate midpointTo(GeoCoordinate other) {
        return GeodeticCalculator.midpoint(this, other);
    }

    /**
     * Calculate a destination point given distance and bearing
     * @param distanceNM Distance in nautical miles
     * @param bearingDeg Bearing in degrees
     * @return Destination coordinate
     */
    public GeoCoordinate destinationPoint(double distanceNM, double bearingDeg) {
        return GeodeticCalculator.destinationPoint(this, distanceNM, bearingDeg);
    }

    /**
     * Interpolate between this coordinate and another
     * Note: This is a simple linear interpolation, not a great circle path.
     * For accurate long-distance interpolation, use GeodeticCalculator.greatCirclePath()
     *
     * @param other Other geographic coordinate
     * @param fraction Fraction between 0 and 1
     * @return Interpolated coordinate
     */
    public GeoCoordinate interpolate(GeoCoordinate other, double fraction) {
        // Clamp fraction between 0 and 1
        fraction = Math.max(0, Math.min(1, fraction));

        double lat = latitude + fraction * (other.latitude - latitude);
        double lon = longitude + fraction * (other.longitude - longitude);
        double alt = altitude + fraction * (other.altitude - altitude);

        return GeoCoordinate.builder()
                .latitude(lat)
                .longitude(lon)
                .altitude(alt)
                .build();

        // Alternative methods using great circle path -- uncomment to use, but less efficient and accurate depending on distance between points, most realistic is the first one
        // List<GeoCoordinate> path = GeodeticCalculator.greatCirclePath(this, other);
        // return path.get((int) Math.floor(fraction * (path.size() - 1)));
        // return GeodeticCalculator.greatCirclePath(this, other, fraction).get(1);
        // return GeodeticCalculator.greatCirclePath(this, other, 2).get(1);
    }

    /**
     * Convert altitude from feet to meters
     * @return Altitude in meters
     */
    public double getAltitudeMeters() {
        return altitude * EarthModel.METERS_PER_FOOT;
    }

    /**
     * Set altitude in meters
     * @param altitudeMeters Altitude in meters
     */
    public void setAltitudeMeters(double altitudeMeters) {
        this.altitude = altitudeMeters / EarthModel.METERS_PER_FOOT;
    }

    /**
     * Check if this point is within a specified distance of another point
     * @param other Other geographic coordinate
     * @param distanceNM Maximum distance in nautical miles
     * @return True if points are within the specified distance
     */
    public boolean isWithinDistance(GeoCoordinate other, double distanceNM) {
        return distanceTo(other) <= distanceNM;
    }

    /**
     * Check if this point is on a great circle path segment (within tolerance)
     * @param pathStart Start of path segment
     * @param pathEnd End of path segment
     * @param toleranceNM Tolerance in nautical miles
     * @return True if point is on the path segment
     */
    public boolean isOnPathSegment(GeoCoordinate pathStart, GeoCoordinate pathEnd, double toleranceNM) {
        return GeodeticCalculator.isPointOnPathSegment(pathStart, pathEnd, this, toleranceNM);
    }

    /**
     * Find the closest point on a great circle path to this point
     * @param pathStart Start of path
     * @param pathEnd End of path
     * @return The closest point on the path
     */
    public GeoCoordinate closestPointOnPath(GeoCoordinate pathStart, GeoCoordinate pathEnd) {
        return GeodeticCalculator.closestPointOnPath(pathStart, pathEnd, this);
    }

    /**
     * Calculate the cross-track distance of this point from a great circle path
     * @param pathStart Start of path
     * @param pathEnd End of path
     * @return Cross-track distance in nautical miles (negative if to the left of the path)
     */
    public double crossTrackDistanceToPath(GeoCoordinate pathStart, GeoCoordinate pathEnd) {
        return GeodeticCalculator.crossTrackDistance(pathStart, pathEnd, this);
    }

    /**
     * Calculate the along-track distance of this point from the start of a great circle path
     * @param pathStart Start of path
     * @param pathEnd End of path
     * @return Along-track distance in nautical miles
     */
    public double alongTrackDistanceOnPath(GeoCoordinate pathStart, GeoCoordinate pathEnd) {
        return GeodeticCalculator.alongTrackDistance(pathStart, pathEnd, this);
    }

    /**
     * Create a GeoCoordinate from ECEF coordinates
     * @param ecef ECEF coordinates
     * @return GeoCoordinate
     */
    public static GeoCoordinate fromECEF(ECEFCoordinate ecef) {
        return ecef.toGeodetic();
    }

    /**
     * Create a GeoCoordinate from ENU coordinates
     * @param enu ENU coordinates
     * @param reference The reference point (origin of ENU)
     * @return GeoCoordinate
     */
    public static GeoCoordinate fromENU(ENUCoordinate enu, GeoCoordinate reference) {
        return enu.toGeodetic(reference);
    }

    /**
     * Format coordinate as a string
     * @return Formatted coordinate string
     */
    @Override
    public String toString() {
        return String.format("%.6f°, %.6f°, %.0f ft", latitude, longitude, altitude);
    }

    public GeoCoordinate withAltitude(double lowerAltitude) {
        return GeoCoordinate.builder()
                .latitude(latitude)
                .longitude(longitude)
                .altitude(lowerAltitude)
                .build();
    }

    public double bearingTo(GeoCoordinate coordinate) {
        return GeodeticCalculator.initialBearing(this, coordinate);
    }
}