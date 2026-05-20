package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

/**
 * Default CARF route standards used when no airspace-specific table is loaded.
 */
public class DefaultCarfRouteSeparationResolver implements CarfRouteSeparationResolver {
    private static final CarfSeparationStandard DEFAULT_STANDARD = CarfSeparationStandard.builder()
            .lateralSeparationNauticalMiles(185218.52000000002 / 1852.0)
            .verticalSeparationFeet(499.0)
            .longitudinalSeparationMinutes(0.0)
            .routeWidthNauticalMiles(100.0)
            .build();

    @Override
    public CarfSeparationStandard resolve(GeoCoordinate point, double lowerAltitudeFeet, double upperAltitudeFeet) {
        return DEFAULT_STANDARD;
    }
}
