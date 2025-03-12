package org.tash.core.routing.search.astar;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.core.routing.raw.RoutingNode;
import org.tash.spatial.SpatialPoint;

import java.util.List;
import java.util.Set;

/**
 * Interface for generating neighbor nodes
 */
public interface NodeGenerator {
    /**
     * Generate neighbor nodes from the current node
     *
     * @param current Current node
     * @param goal Goal point
     * @param constraints Routing constraints
     * @param heuristic Heuristic function
     * @param config A* configuration
     * @return List of neighbor nodes
     */
    List<RoutingNode> generateNeighbors(RoutingNode current, SpatialPoint goal,
                                        Set<RoutingConstraint> constraints,
                                        HeuristicFunction heuristic,
                                        AStarConfig config);
}