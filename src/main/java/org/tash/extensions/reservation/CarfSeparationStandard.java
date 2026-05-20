package org.tash.extensions.reservation;

import lombok.Builder;
import lombok.Data;

/**
 * Separation standards resolved for a CARF route point or segment.
 */
@Data
@Builder
public class CarfSeparationStandard {
    private double lateralSeparationNauticalMiles;
    private double verticalSeparationFeet;
    private double longitudinalSeparationMinutes;
    private double routeWidthNauticalMiles;

    public boolean sameAs(CarfSeparationStandard other) {
        if (other == null) {
            return false;
        }
        return same(lateralSeparationNauticalMiles, other.lateralSeparationNauticalMiles)
                && same(verticalSeparationFeet, other.verticalSeparationFeet)
                && same(longitudinalSeparationMinutes, other.longitudinalSeparationMinutes)
                && same(routeWidthNauticalMiles, other.routeWidthNauticalMiles);
    }

    private boolean same(double first, double second) {
        return Math.abs(first - second) < 0.000001;
    }
}
