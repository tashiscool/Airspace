package org.tash.core.routing.search.astar;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.spatial.SpatialPoint;

import java.util.Set;

/**
 * Default heuristic based on geodetic distance
 */
public class GeodeticHeuristic implements HeuristicFunction {
    @Override
    public double estimate(SpatialPoint current, SpatialPoint goal, Set<RoutingConstraint> constraints) {
        return current.getCoordinate().distanceTo(goal.getCoordinate());
    }
}