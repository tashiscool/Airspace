package org.tash.extensions.geodetic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import org.tash.data.GeoCoordinate;

/**
 * Earth-Centered, Earth-Fixed (ECEF) coordinate system
 * Represents a point in 3D Cartesian coordinates with origin at Earth's center
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ECEFCoordinate {
    /** X coordinate in meters (positive towards Prime Meridian/Greenwich) */
    private double x;
    
    /** Y coordinate in meters (positive towards 90° east longitude) */
    private double y;
    
    /** Z coordinate in meters (positive towards North Pole) */
    private double z;
    
    /**
     * Convert from geodetic coordinates (lat/lon/alt) to ECEF
     * 
     * @param coordinate Geodetic coordinate (latitude, longitude in degrees, altitude in feet)
     * @return ECEF coordinate in meters
     */
    public static ECEFCoordinate fromGeodetic(GeoCoordinate coordinate) {
        double latRad = Math.toRadians(coordinate.getLatitude());
        double lonRad = Math.toRadians(coordinate.getLongitude());
        
        // Convert altitude from feet to meters
        double altMeters = coordinate.getAltitude() * EarthModel.METERS_PER_FOOT;
        
        // Calculate N (radius of curvature in the prime vertical)
        double N = EarthModel.normalRadiusOfCurvature(latRad);
        
        // Calculate ECEF coordinates
        double x = (N + altMeters) * Math.cos(latRad) * Math.cos(lonRad);
        double y = (N + altMeters) * Math.cos(latRad) * Math.sin(lonRad);
        double z = (N * (1 - EarthModel.WGS84_ECCENTRICITY_SQUARED) + altMeters) * Math.sin(latRad);
        
        return new ECEFCoordinate(x, y, z);
    }
    
    /**
     * Convert from ECEF to geodetic coordinates (lat/lon/alt)
     * 
     * @return Geodetic coordinate (latitude, longitude in degrees, altitude in feet)
     */
    public GeoCoordinate toGeodetic() {
        // Implementation of Bowring's method for geodetic conversion
        // (more accurate than simple methods, especially at high altitudes)
        
        // Calculate auxiliary values
        double p = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(z * EarthModel.WGS84_SEMI_MAJOR_AXIS, 
                               p * EarthModel.WGS84_SEMI_MINOR_AXIS);
        
        // Calculate latitude
        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);
        double num = z + EarthModel.WGS84_ECCENTRICITY_SQUARED * 
                   EarthModel.WGS84_SEMI_MINOR_AXIS * sinTheta * sinTheta * sinTheta;
        double denom = p - EarthModel.WGS84_ECCENTRICITY_SQUARED * 
                     EarthModel.WGS84_SEMI_MAJOR_AXIS * cosTheta * cosTheta * cosTheta;
        
        double latRad = Math.atan2(num, denom);
        
        // Calculate longitude
        double lonRad = Math.atan2(y, x);
        
        // Calculate altitude
        double sinLat = Math.sin(latRad);
        double N = EarthModel.normalRadiusOfCurvature(latRad);
        
        // Altitude above ellipsoid in meters
        double altMeters = p / Math.cos(latRad) - N;
        
        // Convert to degrees and feet
        double lat = Math.toDegrees(latRad);
        double lon = Math.toDegrees(lonRad);
        double altFeet = altMeters / EarthModel.METERS_PER_FOOT;
        
        return GeoCoordinate.builder()
                .latitude(lat)
                .longitude(lon)
                .altitude(altFeet)
                .build();
    }
    
    /**
     * Calculate the distance between two ECEF coordinates
     * 
     * @param other Other ECEF coordinate
     * @return Distance in meters
     */
    public double distanceTo(ECEFCoordinate other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        double dz = other.z - this.z;
        
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
}
