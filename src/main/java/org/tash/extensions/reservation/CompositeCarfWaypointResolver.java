package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class CompositeCarfWaypointResolver implements CarfWaypointResolver {
    private final List<CarfWaypointResolver> resolvers;

    public CompositeCarfWaypointResolver(List<CarfWaypointResolver> resolvers) {
        this.resolvers = Collections.unmodifiableList(new ArrayList<>(resolvers));
    }

    public CompositeCarfWaypointResolver(CarfWaypointResolver... resolvers) {
        this(Arrays.asList(resolvers));
    }

    @Override
    public Optional<GeoCoordinate> resolve(String name) {
        for (CarfWaypointResolver resolver : resolvers) {
            Optional<GeoCoordinate> coordinate = resolver.resolve(name);
            if (coordinate.isPresent()) {
                return coordinate;
            }
        }
        return Optional.empty();
    }

    public List<CarfWaypointResolver> resolvers() {
        return resolvers;
    }
}
