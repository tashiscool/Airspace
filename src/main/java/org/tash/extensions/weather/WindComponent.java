package org.tash.extensions.weather;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

/**
 * Represents wind in terms of direction and speed
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class WindComponent {
    /**
     * Wind direction in degrees (0-360, 0 = North, 90 = East)
     */
    private double direction;

    /**
     * Wind speed in knots
     */
    private double speed;

    /**
     * Vertical speed in knots (positive = up)
     */
    private double verticalSpeed;

    /**
     * Convert to a vector representation
     *
     * @return Vector3 object
     */
    public Vector3 toVector() {
        return Vector3.fromDirectionAndSpeed(direction, speed, verticalSpeed);
    }

    /**
     * Get the headwind/tailwind component relative to an aircraft heading
     *
     * @param aircraftHeading Aircraft heading in degrees
     * @return Headwind component in knots (positive = headwind, negative = tailwind)
     */
    public double getHeadwindComponent(double aircraftHeading) {
        double relativeAngle = Math.abs((direction - aircraftHeading + 180) % 360 - 180);
        return speed * Math.cos(Math.toRadians(relativeAngle));
    }

    /**
     * Get the crosswind component relative to an aircraft heading
     *
     * @param aircraftHeading Aircraft heading in degrees
     * @return Crosswind component in knots (positive = from right to left)
     */
    public double getCrosswindComponent(double aircraftHeading) {
        double relativeAngle = (direction - aircraftHeading + 180) % 360 - 180;
        return speed * Math.sin(Math.toRadians(relativeAngle));
    }
}
