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

import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.spatial.SpatialTopologyBackend;
import org.tash.extensions.engine.spatial.SpatialTopologyExtension;

public class ConstraintSpatialIndex {
    private final double cellResolutionDegrees;
    private final int timeBucketMinutes;
    private final int altitudeBucketFeet;
    private final EngineConfig config;
    private final SpatialTopologyExtension topologyExtension;
    private final List<OperationalConstraint> constraints;
    private final Map<String, List<OperationalConstraint>> index = new LinkedHashMap<>();

    public ConstraintSpatialIndex(Collection<OperationalConstraint> constraints) {
        this(constraints, EngineConfig.builder().build());
    }

    public ConstraintSpatialIndex(Collection<OperationalConstraint> constraints, EngineConfig config) {
        this(constraints, config, null);
    }

    public ConstraintSpatialIndex(Collection<OperationalConstraint> constraints, EngineConfig config, SpatialTopologyExtension topologyExtension) {
        EngineConfig safe = config == null ? EngineConfig.builder().build() : config;
        this.config = safe;
        this.cellResolutionDegrees = safe.getIndexCellResolutionDegrees();
        this.timeBucketMinutes = safe.getTimeBucketMinutes();
        this.altitudeBucketFeet = safe.getAltitudeBucketFeet();
        this.topologyExtension = topologyExtension != null && topologyExtension.isAvailable()
                && topologyExtension.supportsDiscreteIndex() ? topologyExtension : null;
        this.constraints = constraints == null ? new ArrayList<>() : new ArrayList<>(constraints);
        for (OperationalConstraint constraint : this.constraints) {
            for (String key : keysFor(constraint)) {
                index.computeIfAbsent(key, ignored -> new ArrayList<>()).add(constraint);
            }
        }
    }

    public List<OperationalConstraint> query(RouteCorridor corridor, TimeWindow window, AltitudeBand band, ProductValidityPolicy validityPolicy) {
        Set<OperationalConstraint> candidates = new HashSet<>();
        if (corridor == null) {
            candidates.addAll(constraints);
        } else {
            boolean usedTopology = false;
            for (String cell : coveringCellsFor(corridor)) {
                for (String key : index.keySet()) {
                    if (key.startsWith("topology:" + cell + ":")) {
                        candidates.addAll(index.getOrDefault(key, Collections.emptyList()));
                        usedTopology = true;
                    }
                }
            }
            for (int lat = bucket(corridor.minLatitude()); lat <= bucket(corridor.maxLatitude()); lat++) {
                for (int lon = bucket(corridor.minLongitude()); lon <= bucket(corridor.maxLongitude()); lon++) {
                    for (String key : index.keySet()) {
                        if (key.startsWith(lat + ":" + lon + ":")) {
                            candidates.addAll(index.getOrDefault(key, Collections.emptyList()));
                        }
                    }
                }
            }
            if (!usedTopology && candidates.isEmpty()) {
                candidates.addAll(constraints);
            }
        }
        List<OperationalConstraint> matches = new ArrayList<>();
        for (OperationalConstraint constraint : candidates) {
            if (!validByPolicy(constraint, window, validityPolicy)) continue;
            if (!timeOverlaps(constraint, window)) continue;
            if (!altitudeOverlaps(constraint, band)) continue;
            if (corridor != null && !bboxOverlaps(constraint, corridor)) continue;
            matches.add(constraint);
        }
        return matches;
    }

    public int candidateCount(RouteCorridor corridor) {
        return query(corridor, null, null, ProductValidityPolicy.INCLUDE_UNKNOWN_VALIDITY).size();
    }

    public int indexCellCount() {
        return index.size();
    }

    public List<String> coveringCellsFor(OperationalConstraint constraint) {
        if (constraint == null || topologyExtension == null) {
            return Collections.emptyList();
        }
        return topologyExtension.coveringCells(constraint.getGeometry(), topologyResolution());
    }

    private List<String> keysFor(OperationalConstraint constraint) {
        if (constraint == null || constraint.getGeometry().isEmpty()) {
            return Collections.singletonList("nonspatial:all:all");
        }
        double minLat = constraint.getGeometry().stream().mapToDouble(org.tash.data.GeoCoordinate::getLatitude).min().orElse(0);
        double maxLat = constraint.getGeometry().stream().mapToDouble(org.tash.data.GeoCoordinate::getLatitude).max().orElse(0);
        double minLon = constraint.getGeometry().stream().mapToDouble(org.tash.data.GeoCoordinate::getLongitude).min().orElse(0);
        double maxLon = constraint.getGeometry().stream().mapToDouble(org.tash.data.GeoCoordinate::getLongitude).max().orElse(0);
        int timeStart = timeBucket(constraint.getStartTime());
        int timeEnd = timeBucket(constraint.getEndTime());
        int altStart = altitudeBucket(constraint.getLowerAltitudeFeet());
        int altEnd = altitudeBucket(constraint.getUpperAltitudeFeet());
        List<String> keys = new ArrayList<>();
        for (int lat = bucket(minLat); lat <= bucket(maxLat); lat++) {
            for (int lon = bucket(minLon); lon <= bucket(maxLon); lon++) {
                for (int time = timeStart; time <= timeEnd; time++) {
                    for (int alt = altStart; alt <= altEnd; alt++) {
                        keys.add(lat + ":" + lon + ":" + time + ":" + alt);
                    }
                }
            }
        }
        for (String cell : coveringCellsFor(constraint)) {
            for (int time = timeStart; time <= timeEnd; time++) {
                for (int alt = altStart; alt <= altEnd; alt++) {
                    keys.add("topology:" + cell + ":" + time + ":" + alt);
                }
            }
        }
        return keys;
    }

    private List<String> coveringCellsFor(RouteCorridor corridor) {
        if (corridor == null || topologyExtension == null) {
            return Collections.emptyList();
        }
        List<GeoCoordinate> points = new ArrayList<>(corridor.getPoints());
        points.add(GeoCoordinate.builder().latitude(corridor.minLatitude()).longitude(corridor.minLongitude()).build());
        points.add(GeoCoordinate.builder().latitude(corridor.minLatitude()).longitude(corridor.maxLongitude()).build());
        points.add(GeoCoordinate.builder().latitude(corridor.maxLatitude()).longitude(corridor.maxLongitude()).build());
        points.add(GeoCoordinate.builder().latitude(corridor.maxLatitude()).longitude(corridor.minLongitude()).build());
        return topologyExtension.coveringCells(points, topologyResolution());
    }

    private int topologyResolution() {
        if (topologyExtension == null) {
            return 1;
        }
        if (topologyExtension.backend() == SpatialTopologyBackend.S2) {
            return config.getS2Level();
        }
        if (topologyExtension.backend() == SpatialTopologyBackend.H3) {
            return config.getH3Resolution();
        }
        return Math.max(1, (int) Math.round(1.0 / Math.max(0.000001, cellResolutionDegrees)));
    }

    private boolean validByPolicy(OperationalConstraint constraint, TimeWindow window, ProductValidityPolicy policy) {
        if (policy == ProductValidityPolicy.INCLUDE_STALE || policy == null) return true;
        if (constraint.getStartTime() == null || constraint.getEndTime() == null) {
            return policy == ProductValidityPolicy.INCLUDE_UNKNOWN_VALIDITY;
        }
        if (window == null || window.getStart() == null) return true;
        return !constraint.getEndTime().isBefore(window.getStart());
    }

    private boolean timeOverlaps(OperationalConstraint c, TimeWindow window) {
        if (window == null || window.getStart() == null || window.getEnd() == null
                || c.getStartTime() == null || c.getEndTime() == null) return true;
        return !c.getEndTime().isBefore(window.getStart()) && !c.getStartTime().isAfter(window.getEnd());
    }

    private boolean altitudeOverlaps(OperationalConstraint c, AltitudeBand band) {
        if (band == null) return true;
        return c.getUpperAltitudeFeet() >= band.getLowerFeet() && c.getLowerAltitudeFeet() <= band.getUpperFeet();
    }

    private boolean bboxOverlaps(OperationalConstraint constraint, RouteCorridor corridor) {
        if (constraint.getGeometry().isEmpty()) return true;
        double minLat = constraint.getGeometry().stream().mapToDouble(org.tash.data.GeoCoordinate::getLatitude).min().orElse(0);
        double maxLat = constraint.getGeometry().stream().mapToDouble(org.tash.data.GeoCoordinate::getLatitude).max().orElse(0);
        double minLon = constraint.getGeometry().stream().mapToDouble(org.tash.data.GeoCoordinate::getLongitude).min().orElse(0);
        double maxLon = constraint.getGeometry().stream().mapToDouble(org.tash.data.GeoCoordinate::getLongitude).max().orElse(0);
        return maxLat >= corridor.minLatitude() && minLat <= corridor.maxLatitude()
                && maxLon >= corridor.minLongitude() && minLon <= corridor.maxLongitude();
    }

    private int bucket(double value) {
        return (int) Math.floor(value / cellResolutionDegrees);
    }

    private int timeBucket(ZonedDateTime time) {
        if (time == null) return 0;
        long epochMinutes = time.toEpochSecond() / 60;
        return (int) (epochMinutes / Math.max(1, timeBucketMinutes));
    }

    private int altitudeBucket(double feet) {
        return (int) Math.floor(feet / Math.max(1, altitudeBucketFeet));
    }
}
