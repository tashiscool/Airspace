package org.tash.extensions.geodetic;

import lombok.*;
import org.tash.data.GeoCoordinate;

/**
 * East-North-Up (ENU) coordinate system
 * Local tangent plane coordinates with origin at a reference point
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ENUCoordinate {
    /** East coordinate in meters */
    private double east;
    
    /** North coordinate in meters */
    private double north;
    
    /** Up coordinate in meters */
    private double up;
    
    /**
     * Convert from geodetic to ENU coordinates
     * 
     * @param coordinate The point to convert
     * @param reference The reference point (origin of ENU)
     * @return ENU coordinate
     */
    public static ENUCoordinate fromGeodetic(GeoCoordinate coordinate, GeoCoordinate reference) {
        // First convert both coordinates to ECEF
        ECEFCoordinate ecef = ECEFCoordinate.fromGeodetic(coordinate);
        ECEFCoordinate ecefRef = ECEFCoordinate.fromGeodetic(reference);
        
        return fromECEF(ecef, ecefRef, reference);
    }
    
    /**
     * Convert from ECEF to ENU coordinates
     * 
     * @param ecef The ECEF coordinate to convert
     * @param ecefReference The ECEF of the reference point (origin of ENU)
     * @param geoReference The geodetic reference point
     * @return ENU coordinate
     */
    public static ENUCoordinate fromECEF(ECEFCoordinate ecef, ECEFCoordinate ecefReference, 
                                        GeoCoordinate geoReference) {
        // Calculate ECEF vector from reference to point
        double dx = ecef.getX() - ecefReference.getX();
        double dy = ecef.getY() - ecefReference.getY();
        double dz = ecef.getZ() - ecefReference.getZ();
        
        // Get reference point lat/lon in radians
        double latRad = Math.toRadians(geoReference.getLatitude());
        double lonRad = Math.toRadians(geoReference.getLongitude());
        
        // Compute sines and cosines
        double sinLat = Math.sin(latRad);
        double cosLat = Math.cos(latRad);
        double sinLon = Math.sin(lonRad);
        double cosLon = Math.cos(lonRad);
        
        // Compute ENU coordinates
        double east = -sinLon * dx + cosLon * dy;
        double north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
        double up = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;
        
        return new ENUCoordinate(east, north, up);
    }
    
    /**
     * Convert ENU to geodetic coordinates
     * 
     * @param reference The reference point (origin of ENU)
     * @return Geodetic coordinate
     */
    public GeoCoordinate toGeodetic(GeoCoordinate reference) {
        // Convert reference to ECEF
        ECEFCoordinate ecefRef = ECEFCoordinate.fromGeodetic(reference);
        
        // Convert ENU to ECEF
        ECEFCoordinate ecef = toECEF(ecefRef, reference);
        
        // Convert ECEF to geodetic
        return ecef.toGeodetic();
    }
    
    /**
     * Convert ENU to ECEF coordinates
     * 
     * @param ecefReference The ECEF of the reference point
     * @param geoReference The geodetic reference point
     * @return ECEF coordinate
     */
    public ECEFCoordinate toECEF(ECEFCoordinate ecefReference, GeoCoordinate geoReference) {
        // Get reference point lat/lon in radians
        double latRad = Math.toRadians(geoReference.getLatitude());
        double lonRad = Math.toRadians(geoReference.getLongitude());
        
        // Compute sines and cosines
        double sinLat = Math.sin(latRad);
        double cosLat = Math.cos(latRad);
        double sinLon = Math.sin(lonRad);
        double cosLon = Math.cos(lonRad);
        
        // ENU to ECEF transformation
        double dx = -sinLon * east - sinLat * cosLon * north + cosLat * cosLon * up;
        double dy = cosLon * east - sinLat * sinLon * north + cosLat * sinLon * up;
        double dz = cosLat * north + sinLat * up;
        
        // Add reference point
        double x = dx + ecefReference.getX();
        double y = dy + ecefReference.getY();
        double z = dz + ecefReference.getZ();
        
        return new ECEFCoordinate(x, y, z);
    }
    
    /**
     * Calculate the distance between two ENU coordinates
     * 
     * @param other Other ENU coordinate
     * @return Distance in meters
     */
    public double distanceTo(ENUCoordinate other) {
        double dEast = other.east - this.east;
        double dNorth = other.north - this.north;
        double dUp = other.up - this.up;
        
        return Math.sqrt(dEast * dEast + dNorth * dNorth + dUp * dUp);
    }
}