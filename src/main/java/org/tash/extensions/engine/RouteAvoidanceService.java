package org.tash.extensions.engine;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.routing.OperationalRoutePlanRequest;
import org.tash.extensions.routing.OperationalRoutePlanResult;
import org.tash.extensions.routing.RoutePlanningConstraint;
import org.tash.extensions.routing.RouteReplanningService;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Suggests deterministic deviation corridors from normalized operational
 * constraints. It is intentionally adapter-free so certified routing solvers can
 * be plugged in later without changing engine callers.
 */
public class RouteAvoidanceService {
    private final RouteReplanningService routeReplanningService;

    public RouteAvoidanceService() {
        this(new RouteReplanningService());
    }

    public RouteAvoidanceService(RouteReplanningService routeReplanningService) {
        this.routeReplanningService = routeReplanningService == null ? new RouteReplanningService() : routeReplanningService;
    }

    public OperationalRoutePlanResult suggest(OperationalRoutePlanRequest request) {
        return routeReplanningService.plan(request == null ? OperationalRoutePlanRequest.builder().build() : request);
    }

    public OperationalRoutePlanResult suggest(List<GeoCoordinate> route,
                                              List<OperationalConstraint> constraints,
                                              ZonedDateTime decisionTime) {
        List<RoutePlanningConstraint> planningConstraints = new ArrayList<>();
        for (OperationalConstraint constraint : constraints == null ? Collections.<OperationalConstraint>emptyList() : constraints) {
            planningConstraints.add(RoutePlanningConstraint.from(constraint));
        }
        return suggest(OperationalRoutePlanRequest.builder()
                .originalRoute(route == null ? Collections.<GeoCoordinate>emptyList() : route)
                .constraints(planningConstraints)
                .decisionTime(decisionTime)
                .build());
    }
}
