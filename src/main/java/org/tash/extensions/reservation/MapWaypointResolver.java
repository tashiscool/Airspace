package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.util.Collections;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

public class MapWaypointResolver implements CarfWaypointResolver {
    private final Map<String, GeoCoordinate> waypoints;

    public MapWaypointResolver(Map<String, GeoCoordinate> waypoints) {
        Map<String, GeoCoordinate> normalized = new HashMap<>();
        for (Map.Entry<String, GeoCoordinate> entry : waypoints.entrySet()) {
            normalized.put(normalize(entry.getKey()), entry.getValue());
        }
        this.waypoints = Collections.unmodifiableMap(normalized);
    }

    @Override
    public Optional<GeoCoordinate> resolve(String name) {
        return Optional.ofNullable(waypoints.get(normalize(name)));
    }

    private String normalize(String name) {
        return name.toUpperCase(Locale.US).replaceAll("[^A-Z0-9]", "");
    }
}
