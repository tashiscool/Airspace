package org.tash.core.routing.search.astar;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.spatial.SpatialPoint;

import java.util.Set;

/**
 * Interface for custom heuristic functions
 */
public interface HeuristicFunction {
    /**
     * Estimate the cost from current to goal
     *
     * @param current Current point
     * @param goal Goal point
     * @param constraints Routing constraints
     * @return Estimated cost
     */
    double estimate(SpatialPoint current, SpatialPoint goal, Set<RoutingConstraint> constraints);
}