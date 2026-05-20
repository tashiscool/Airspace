package org.tash.extensions.engine;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class ConstraintIndex {
    private final List<OperationalConstraint> constraints;
    private final List<IndexedConstraint> indexedConstraints;
    private final Map<String, List<IndexedConstraint>> gridIndex;
    private final List<IndexedConstraint> nonSpatialConstraints;

    public ConstraintIndex(Collection<OperationalConstraint> constraints) {
        this.constraints = constraints == null ? new ArrayList<>() : new ArrayList<>(constraints);
        this.indexedConstraints = new ArrayList<>();
        this.gridIndex = new LinkedHashMap<>();
        this.nonSpatialConstraints = new ArrayList<>();
        for (OperationalConstraint constraint : this.constraints) {
            IndexedConstraint indexed = new IndexedConstraint(constraint);
            this.indexedConstraints.add(indexed);
            index(indexed);
        }
    }

    public List<OperationalConstraint> query(ZonedDateTime start, ZonedDateTime end, double lowerFeet, double upperFeet) {
        List<OperationalConstraint> matches = new ArrayList<>();
        for (IndexedConstraint indexed : indexedConstraints) {
            if (timeOverlaps(indexed.constraint, start, end) && altitudeOverlaps(indexed.constraint, lowerFeet, upperFeet)) {
                matches.add(indexed.constraint);
            }
        }
        return matches;
    }

    public List<OperationalConstraint> query(RouteCorridor corridor, TimeWindow window, AltitudeBand band) {
        List<OperationalConstraint> matches = new ArrayList<>();
        for (IndexedConstraint indexed : candidates(corridor)) {
            if (!timeOverlaps(indexed.constraint, window == null ? null : window.getStart(),
                    window == null ? null : window.getEnd())) {
                continue;
            }
            if (!altitudeOverlaps(indexed.constraint,
                    band == null ? Double.NEGATIVE_INFINITY : band.getLowerFeet(),
                    band == null ? Double.POSITIVE_INFINITY : band.getUpperFeet())) {
                continue;
            }
            if (corridor == null || indexed.overlaps(corridor)) {
                matches.add(indexed.constraint);
            }
        }
        return matches;
    }

    public int getGridCellCount() {
        return gridIndex.size();
    }

    public Map<Integer, List<OperationalConstraint>> queryRouteSegments(List<org.tash.data.GeoCoordinate> route,
                                                                        TimeWindow window,
                                                                        AltitudeBand band,
                                                                        double segmentBufferNauticalMiles) {
        Map<Integer, List<OperationalConstraint>> matches = new LinkedHashMap<>();
        if (route == null || route.size() < 2) {
            return matches;
        }
        for (int i = 0; i < route.size() - 1; i++) {
            List<org.tash.data.GeoCoordinate> segment = new ArrayList<>();
            segment.add(route.get(i));
            segment.add(route.get(i + 1));
            List<OperationalConstraint> segmentMatches = query(RouteCorridor.builder()
                    .points(segment)
                    .bufferNauticalMiles(segmentBufferNauticalMiles)
                    .build(), window, band);
            if (!segmentMatches.isEmpty()) {
                matches.put(i, segmentMatches);
            }
        }
        return matches;
    }

    private boolean timeOverlaps(OperationalConstraint c, ZonedDateTime start, ZonedDateTime end) {
        if (start == null || end == null || c.getStartTime() == null || c.getEndTime() == null) return true;
        return !c.getEndTime().isBefore(start) && !c.getStartTime().isAfter(end);
    }

    private boolean altitudeOverlaps(OperationalConstraint c, double lower, double upper) {
        return c.getUpperAltitudeFeet() >= lower && c.getLowerAltitudeFeet() <= upper;
    }

    private void index(IndexedConstraint indexed) {
        if (Double.isInfinite(indexed.minLat)) {
            nonSpatialConstraints.add(indexed);
            return;
        }
        for (int lat = bucket(indexed.minLat); lat <= bucket(indexed.maxLat); lat++) {
            for (int lon = bucket(indexed.minLon); lon <= bucket(indexed.maxLon); lon++) {
                gridIndex.computeIfAbsent(cell(lat, lon), ignored -> new ArrayList<>()).add(indexed);
            }
        }
    }

    private List<IndexedConstraint> candidates(RouteCorridor corridor) {
        if (corridor == null || gridIndex.isEmpty()) {
            return indexedConstraints;
        }
        Set<IndexedConstraint> candidates = new HashSet<>(nonSpatialConstraints);
        for (int lat = bucket(corridor.minLatitude()); lat <= bucket(corridor.maxLatitude()); lat++) {
            for (int lon = bucket(corridor.minLongitude()); lon <= bucket(corridor.maxLongitude()); lon++) {
                candidates.addAll(gridIndex.getOrDefault(cell(lat, lon), Collections.emptyList()));
            }
        }
        return new ArrayList<>(candidates);
    }

    private int bucket(double value) {
        return (int) Math.floor(value);
    }

    private String cell(int lat, int lon) {
        return lat + ":" + lon;
    }

    private static final class IndexedConstraint {
        private final OperationalConstraint constraint;
        private final double minLat;
        private final double maxLat;
        private final double minLon;
        private final double maxLon;

        private IndexedConstraint(OperationalConstraint constraint) {
            this.constraint = constraint;
            double minLatValue = Double.POSITIVE_INFINITY;
            double maxLatValue = Double.NEGATIVE_INFINITY;
            double minLonValue = Double.POSITIVE_INFINITY;
            double maxLonValue = Double.NEGATIVE_INFINITY;
            if (constraint != null && constraint.getGeometry() != null) {
                for (org.tash.data.GeoCoordinate point : constraint.getGeometry()) {
                    if (point == null) continue;
                    minLatValue = Math.min(minLatValue, point.getLatitude());
                    maxLatValue = Math.max(maxLatValue, point.getLatitude());
                    minLonValue = Math.min(minLonValue, point.getLongitude());
                    maxLonValue = Math.max(maxLonValue, point.getLongitude());
                }
            }
            this.minLat = minLatValue;
            this.maxLat = maxLatValue;
            this.minLon = minLonValue;
            this.maxLon = maxLonValue;
        }

        private boolean overlaps(RouteCorridor corridor) {
            if (Double.isInfinite(minLat) || corridor == null) {
                return true;
            }
            return maxLat >= corridor.minLatitude()
                    && minLat <= corridor.maxLatitude()
                    && maxLon >= corridor.minLongitude()
                    && minLon <= corridor.maxLongitude();
        }
    }
}
