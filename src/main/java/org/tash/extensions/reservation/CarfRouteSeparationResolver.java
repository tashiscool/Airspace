package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

/**
 * Resolves CARF separation standards for a route point and altitude block.
 */
public interface CarfRouteSeparationResolver {
    CarfSeparationStandard resolve(GeoCoordinate point, double lowerAltitudeFeet, double upperAltitudeFeet);
}
