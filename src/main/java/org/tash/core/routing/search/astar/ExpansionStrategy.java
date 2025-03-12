package org.tash.core.routing.search.astar;

import org.tash.spatial.SpatialPoint;

import java.util.List;

/**
 * Interface for strategies to expand nodes
 */
public interface ExpansionStrategy {
    /**
     * Expand from a given point to generate candidate neighbors
     *
     * @param current Current point
     * @param goal Goal point
     * @param config A* configuration
     * @return List of candidate points
     */
    List<SpatialPoint> expandFrom(SpatialPoint current, SpatialPoint goal, AStarConfig config);
}