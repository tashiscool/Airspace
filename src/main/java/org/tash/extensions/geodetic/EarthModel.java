package org.tash.extensions.geodetic;

/**
 * Constants and parameters for the WGS84 Earth ellipsoid model
 */
public final class EarthModel {
    /** Semi-major axis of WGS84 ellipsoid in meters */
    public static final double WGS84_SEMI_MAJOR_AXIS = 6378137.0;
    
    /** Semi-minor axis of WGS84 ellipsoid in meters */
    public static final double WGS84_SEMI_MINOR_AXIS = 6356752.314245;
    
    /** Flattening of WGS84 ellipsoid */
    public static final double WGS84_FLATTENING = 1.0 / 298.257223563;
    
    /** Eccentricity of WGS84 ellipsoid */
    public static final double WGS84_ECCENTRICITY = Math.sqrt(1.0 - Math.pow(WGS84_SEMI_MINOR_AXIS / WGS84_SEMI_MAJOR_AXIS, 2));
    
    /** Square of eccentricity */
    public static final double WGS84_ECCENTRICITY_SQUARED = Math.pow(WGS84_ECCENTRICITY, 2);
    
    /** Mean radius of Earth in meters */
    public static final double MEAN_EARTH_RADIUS_METERS = 6371008.8;
    
    /** Mean radius of Earth in nautical miles */
    public static final double MEAN_EARTH_RADIUS_NM = 3440.065;
    
    /** Nautical mile in meters */
    public static final double NAUTICAL_MILE_IN_METERS = 1852.0;
    
    /** Meters per foot */
    public static final double METERS_PER_FOOT = 0.3048;
    
    /**
     * Meridional radius of curvature at a specific latitude
     * @param latitudeRad Latitude in radians
     * @return Radius of curvature in meters
     */
    public static double meridionalRadiusOfCurvature(double latitudeRad) {
        double sinLat = Math.sin(latitudeRad);
        double denom = Math.sqrt(1 - WGS84_ECCENTRICITY_SQUARED * sinLat * sinLat);
        return WGS84_SEMI_MAJOR_AXIS * (1 - WGS84_ECCENTRICITY_SQUARED) / (denom * denom * denom);
    }
    
    /**
     * Normal radius of curvature (perpendicular to meridian) at a specific latitude
     * @param latitudeRad Latitude in radians
     * @return Radius of curvature in meters
     */
    public static double normalRadiusOfCurvature(double latitudeRad) {
        double sinLat = Math.sin(latitudeRad);
        double denom = Math.sqrt(1 - WGS84_ECCENTRICITY_SQUARED * sinLat * sinLat);
        return WGS84_SEMI_MAJOR_AXIS / denom;
    }
    
    /**
     * Radius of curvature in the direction of a given azimuth
     * @param latitudeRad Latitude in radians
     * @param azimuthRad Azimuth in radians
     * @return Radius of curvature in meters
     */
    public static double radiusOfCurvature(double latitudeRad, double azimuthRad) {
        double m = meridionalRadiusOfCurvature(latitudeRad);
        double n = normalRadiusOfCurvature(latitudeRad);
        double cosAz = Math.cos(azimuthRad);
        double sinAz = Math.sin(azimuthRad);
        
        return (m * n) / (m * sinAz * sinAz + n * cosAz * cosAz);
    }
    
    private EarthModel() {
        // Private constructor to prevent instantiation
    }
}