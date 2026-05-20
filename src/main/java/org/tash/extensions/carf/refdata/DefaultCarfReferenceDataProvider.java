package org.tash.extensions.carf.refdata;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.reservation.DefaultCarfWaypointResolver;

import java.util.Optional;

public class DefaultCarfReferenceDataProvider implements CarfReferenceDataProvider {
    private final DefaultCarfWaypointResolver resolver = new DefaultCarfWaypointResolver();

    @Override
    public Optional<GeoCoordinate> resolveFixOrNavaid(String id) {
        return resolver.resolve(id);
    }
}
