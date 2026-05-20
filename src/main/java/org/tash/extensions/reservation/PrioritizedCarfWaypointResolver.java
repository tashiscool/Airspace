package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

/**
 * Dependency-free version of the legacy DAFIF duplicate-navaid selection rule.
 */
public class PrioritizedCarfWaypointResolver implements CarfWaypointResolver {
    private static final List<String> COUNTRY_PRIORITY = Arrays.asList("K", "PA", "PH", "TI", "TJ");

    private final Map<String, List<CarfWaypointCandidate>> candidates;

    public PrioritizedCarfWaypointResolver(List<CarfWaypointCandidate> candidates) {
        Map<String, List<CarfWaypointCandidate>> grouped = new HashMap<>();
        for (CarfWaypointCandidate candidate : candidates) {
            grouped.computeIfAbsent(normalize(candidate.getIdentifier()), ignored -> new ArrayList<>())
                    .add(candidate);
        }
        Map<String, List<CarfWaypointCandidate>> immutable = new HashMap<>();
        for (Map.Entry<String, List<CarfWaypointCandidate>> entry : grouped.entrySet()) {
            List<CarfWaypointCandidate> sorted = new ArrayList<>(entry.getValue());
            sorted.sort(Comparator.comparingInt(this::priority));
            immutable.put(entry.getKey(), Collections.unmodifiableList(sorted));
        }
        this.candidates = Collections.unmodifiableMap(immutable);
    }

    @Override
    public Optional<GeoCoordinate> resolve(String name) {
        List<CarfWaypointCandidate> matches = candidates.get(normalize(name));
        if (matches == null || matches.isEmpty()) {
            return Optional.empty();
        }
        return Optional.ofNullable(matches.get(0).getCoordinate());
    }

    public Optional<CarfWaypointCandidate> resolveCandidate(String name) {
        List<CarfWaypointCandidate> matches = candidates.get(normalize(name));
        if (matches == null || matches.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(matches.get(0));
    }

    public List<CarfWaypointCandidate> candidates(String name) {
        return candidates.getOrDefault(normalize(name), Collections.emptyList());
    }

    private int priority(CarfWaypointCandidate candidate) {
        if (candidate.isPreferred()) {
            return -1;
        }
        String country = normalizeCountry(candidate.getCountryCode());
        for (int i = 0; i < COUNTRY_PRIORITY.size(); i++) {
            String preferredCountry = COUNTRY_PRIORITY.get(i);
            if ("K".equals(preferredCountry) && country.startsWith("K")) {
                return i;
            }
            if (preferredCountry.equals(country)) {
                return i;
            }
        }
        return COUNTRY_PRIORITY.size();
    }

    private String normalize(String value) {
        return value == null ? "" : value.toUpperCase(Locale.US).replaceAll("[^A-Z0-9]", "");
    }

    private String normalizeCountry(String value) {
        return value == null ? "" : value.trim().toUpperCase(Locale.US);
    }
}
