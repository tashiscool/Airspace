package org.tash.extensions.carf.refdata;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.reservation.CarfWaypointResolver;

import java.util.Optional;

public class WaypointReferenceDataProvider implements CarfReferenceDataProvider {
    private final CarfWaypointResolver resolver;

    public WaypointReferenceDataProvider(CarfWaypointResolver resolver) {
        this.resolver = resolver;
    }

    @Override
    public Optional<GeoCoordinate> resolveFixOrNavaid(String id) {
        return resolver.resolve(id);
    }

    public CarfWaypointResolver asWaypointResolver() {
        return resolver::resolve;
    }
}
