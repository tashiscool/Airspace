package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.util.Optional;

public interface CarfWaypointResolver {
    Optional<GeoCoordinate> resolve(String name);
}
