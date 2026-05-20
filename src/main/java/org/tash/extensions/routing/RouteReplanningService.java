package org.tash.extensions.routing;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.DecisionSourceRef;
import org.tash.extensions.engine.OperationalGeometryService;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class RouteReplanningService {
    private final OperationalGeometryService geometryService;

    public RouteReplanningService() {
        this(new OperationalGeometryService());
    }

    public RouteReplanningService(OperationalGeometryService geometryService) {
        this.geometryService = geometryService == null ? new OperationalGeometryService() : geometryService;
    }

    public OperationalRoutePlanResult plan(OperationalRoutePlanRequest request) {
        OperationalRoutePlanRequest safe = request == null ? OperationalRoutePlanRequest.builder().build() : request;
        List<GeoCoordinate> route = new ArrayList<>(safe.getOriginalRoute());
        if (route.size() < 2) {
            return OperationalRoutePlanResult.builder()
                    .originalRoute(route)
                    .blocked(true)
                    .rationale("Route replanning requires at least two route points")
                    .build();
        }
        List<RoutePlanningConstraint> blocking = blockingConstraints(route, safe.getConstraints(), safe.getCorridorWidthNauticalMiles());
        if (blocking.isEmpty()) {
            return OperationalRoutePlanResult.builder()
                    .originalRoute(route)
                    .blocked(false)
                    .rationale("Original route has no blocking constraints")
                    .build();
        }
        List<RouteCandidate> candidates = new ArrayList<>();
        RoutePlanningConstraint primary = blocking.get(0);
        candidates.add(candidate("north-east", route, primary, blocking, 1.0, safe.getCorridorWidthNauticalMiles()));
        candidates.add(candidate("south-west", route, primary, blocking, -1.0, safe.getCorridorWidthNauticalMiles()));
        candidates.removeIf(candidate -> !candidate.getRemainingConstraintIds().isEmpty());
        candidates.sort(Comparator.comparingDouble(RouteCandidate::getCost));
        if (candidates.size() > safe.getMaxCandidates()) {
            candidates = new ArrayList<>(candidates.subList(0, safe.getMaxCandidates()));
        }
        List<DecisionSourceRef> refs = new ArrayList<>();
        refs.add(DecisionSourceRef.builder().type("route-planning").id(primary.getId()).description(primary.getRationale()).build());
        return OperationalRoutePlanResult.builder()
                .originalRoute(route)
                .candidates(candidates)
                .blocked(candidates.isEmpty())
                .rationale(candidates.isEmpty()
                        ? "No deterministic alternate route avoided the blocking constraints"
                        : "Generated deterministic alternate route candidates around blocking constraints")
                .decisionTraceRefs(refs)
                .build();
    }

    private List<RoutePlanningConstraint> blockingConstraints(List<GeoCoordinate> route,
                                                              List<RoutePlanningConstraint> constraints,
                                                              double corridorWidth) {
        List<RoutePlanningConstraint> blocking = new ArrayList<>();
        for (RoutePlanningConstraint constraint : constraints) {
            if (constraint.getGeometry().isEmpty()) {
                continue;
            }
            if (!geometryService.routeSegmentsImpacted(route, constraint.getGeometry(), corridorWidth).isEmpty()) {
                blocking.add(constraint);
            }
        }
        return blocking;
    }

    private RouteCandidate candidate(String id,
                                     List<GeoCoordinate> route,
                                     RoutePlanningConstraint primary,
                                     List<RoutePlanningConstraint> constraints,
                                     double direction,
                                     double corridorWidth) {
        double[] box = bbox(primary.getGeometry());
        double offset = Math.max(0.25, corridorWidth / 60.0 + 0.35);
        GeoCoordinate start = route.get(0);
        GeoCoordinate end = route.get(route.size() - 1);
        double detourLat = direction > 0 ? box[1] + offset : box[0] - offset;
        List<GeoCoordinate> points = new ArrayList<>();
        points.add(start);
        points.add(GeoCoordinate.builder().latitude(detourLat).longitude(start.getLongitude()).altitude(start.getAltitude()).build());
        points.add(GeoCoordinate.builder().latitude(detourLat).longitude(end.getLongitude()).altitude(end.getAltitude()).build());
        points.add(end);

        List<String> avoided = new ArrayList<>();
        List<String> remaining = new ArrayList<>();
        for (RoutePlanningConstraint constraint : constraints) {
            if (constraint.getGeometry().isEmpty()) {
                continue;
            }
            if (geometryService.routeSegmentsImpacted(points, constraint.getGeometry(), corridorWidth).isEmpty()) {
                avoided.add(constraint.getId());
            } else {
                remaining.add(constraint.getId());
            }
        }
        return RouteCandidate.builder()
                .id(id)
                .points(points)
                .cost(distance(points) + remaining.size() * 10000.0)
                .avoidedConstraintIds(avoided)
                .remainingConstraintIds(remaining)
                .rationale("Dogleg route candidate generated around " + primary.getId())
                .build();
    }

    private double[] bbox(List<GeoCoordinate> points) {
        double minLat = Double.POSITIVE_INFINITY;
        double maxLat = Double.NEGATIVE_INFINITY;
        double minLon = Double.POSITIVE_INFINITY;
        double maxLon = Double.NEGATIVE_INFINITY;
        for (GeoCoordinate point : points) {
            minLat = Math.min(minLat, point.getLatitude());
            maxLat = Math.max(maxLat, point.getLatitude());
            minLon = Math.min(minLon, point.getLongitude());
            maxLon = Math.max(maxLon, point.getLongitude());
        }
        return new double[]{minLat, maxLat, minLon, maxLon};
    }

    private double distance(List<GeoCoordinate> points) {
        double total = 0.0;
        for (int i = 0; i + 1 < points.size(); i++) {
            total += points.get(i).distanceTo(points.get(i + 1));
        }
        return total;
    }
}
