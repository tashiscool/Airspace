package org.tash.data;

import lombok.*;
import org.tash.spatial.SpatialPoint;

/**
     * Vector in 3D space
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class Vector3D {
    private double x;
    private double y;
    private double z;

    public Vector3D(SpatialPoint s) {
        this.x = s.getCoordinate().getLatitude();
        this.y = s.getCoordinate().getLatitude();
        this.z = s.getCoordinate().getAltitude();
    }

    /**
     * Add another vector
     */
    public Vector3D add(Vector3D other) {
        return new Vector3D(x + other.x, y + other.y, z + other.z);
    }

    /**
     * Subtract another vector
     */
    public Vector3D subtract(Vector3D other) {
        return new Vector3D(x - other.x, y - other.y, z - other.z);
    }

    /**
     * Multiply by a scalar
     */
    public Vector3D multiply(double scalar) {
        return new Vector3D(x * scalar, y * scalar, z * scalar);
    }

    /**
     * Calculate dot product
     */
    public double dotProduct(Vector3D other) {
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * Calculate magnitude squared
     */
    public double magnitudeSquared() {
        return x * x + y * y + z * z;
    }

    /**
     * Calculate magnitude
     */
    public double magnitude() {
        return Math.sqrt(magnitudeSquared());
    }

    /**
     * Convert from GeoCoordinate
     */
    public static Vector3D fromGeoCoordinate(GeoCoordinate coord) {
        // Convert from latitude/longitude to a local Cartesian coordinate system
        double earthRadius = 3440.065; // Earth radius in nautical miles

        // Convert latitude and longitude to radians
        double lat = Math.toRadians(coord.getLatitude());
        double lon = Math.toRadians(coord.getLongitude());

        // Convert to Cartesian coordinates
        double x = earthRadius * Math.cos(lat) * Math.cos(lon);
        double y = earthRadius * Math.cos(lat) * Math.sin(lon);
        double z = coord.getAltitude(); // Altitude in feet

        return new Vector3D(x, y, z);
    }
}