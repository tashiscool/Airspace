package org.tash.extensions.weather;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

/**
 * Represents a 3D vector for wind and other vector fields
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class Vector3 {
    /**
     * North component in knots
     */
    private double north;

    /**
     * East component in knots
     */
    private double east;

    /**
     * Vertical component in knots (positive = up)
     */
    private double vertical;

    /**
     * Get the magnitude (speed) of the vector
     *
     * @return The magnitude in knots
     */
    public double getMagnitude() {
        return Math.sqrt(north * north + east * east + vertical * vertical);
    }

    /**
     * Get the horizontal magnitude (speed) of the vector
     *
     * @return The horizontal magnitude in knots
     */
    public double getHorizontalMagnitude() {
        return Math.sqrt(north * north + east * east);
    }

    /**
     * Get the direction of the vector in degrees (meteorological convention)
     *
     * @return The direction in degrees (0-360, 0 = North, 90 = East)
     */
    public double getDirection() {
        double angle = Math.toDegrees(Math.atan2(east, north));
        return (angle + 360) % 360;
    }

    /**
     * Convert to a wind component representation (direction and speed)
     *
     * @return WindComponent object
     */
    public WindComponent toWindComponent() {
        return WindComponent.builder()
                .direction(getDirection())
                .speed(getHorizontalMagnitude())
                .verticalSpeed(vertical)
                .build();
    }

    /**
     * Add another vector to this one
     *
     * @param other The vector to add
     * @return The sum vector
     */
    public Vector3 add(Vector3 other) {
        return Vector3.builder()
                .north(this.north + other.north)
                .east(this.east + other.east)
                .vertical(this.vertical + other.vertical)
                .build();
    }

    /**
     * Subtract another vector from this one
     *
     * @param other The vector to subtract
     * @return The difference vector
     */
    public Vector3 subtract(Vector3 other) {
        return Vector3.builder()
                .north(this.north - other.north)
                .east(this.east - other.east)
                .vertical(this.vertical - other.vertical)
                .build();
    }

    /**
     * Multiply the vector by a scalar
     *
     * @param scalar The scalar value
     * @return The scaled vector
     */
    public Vector3 multiply(double scalar) {
        return Vector3.builder()
                .north(this.north * scalar)
                .east(this.east * scalar)
                .vertical(this.vertical * scalar)
                .build();
    }

    /**
     * Calculate the dot product with another vector
     *
     * @param other The other vector
     * @return The dot product
     */
    public double dot(Vector3 other) {
        return this.north * other.north + this.east * other.east + this.vertical * other.vertical;
    }

    /**
     * Create a Vector3 from direction and speed
     *
     * @param direction     Wind direction in degrees (0-360, 0 = North, 90 = East)
     * @param speed         Wind speed in knots
     * @param verticalSpeed Vertical speed in knots (positive = up)
     * @return The corresponding Vector3
     */
    public static Vector3 fromDirectionAndSpeed(double direction, double speed, double verticalSpeed) {
        double dirRad = Math.toRadians(direction);
        double north = speed * Math.cos(dirRad);
        double east = speed * Math.sin(dirRad);

        return Vector3.builder()
                .north(north)
                .east(east)
                .vertical(verticalSpeed)
                .build();
    }
}
