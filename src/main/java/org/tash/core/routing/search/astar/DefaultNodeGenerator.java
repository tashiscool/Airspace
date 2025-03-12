package org.tash.core.routing.search.astar;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.core.routing.raw.RoutingNode;
import org.tash.spatial.SpatialPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Default node generator using a grid-based approach
 */
public class DefaultNodeGenerator implements NodeGenerator {
    @Override
    public List<RoutingNode> generateNeighbors(RoutingNode current, SpatialPoint goal,
                                               Set<RoutingConstraint> constraints,
                                               HeuristicFunction heuristic,
                                               AStarConfig config) {
        List<RoutingNode> neighbors = new ArrayList<>();

        // Get the expansion strategy
        ExpansionStrategy strategy = config.getExpansionStrategy();

        // Generate candidate points using the expansion strategy
        List<SpatialPoint> candidates = strategy.expandFrom(current.getPoint(), goal, config);

        // Create nodes for each candidate
        for (SpatialPoint candidate : candidates) {
            // Check for hard constraints
            boolean violatesHardConstraint = false;
            for (RoutingConstraint constraint : constraints) {
                if (constraint.isHardConstraint() && constraint.isViolated(current.getPoint(), candidate)) {
                    violatesHardConstraint = true;
                    break;
                }
            }

            if (violatesHardConstraint) {
                continue;
            }

            // Calculate g-cost (cost from start to this node)
            double segmentCost = calculateSegmentCost(current.getPoint(), candidate, constraints);
            double gCost = current.getGCost() + segmentCost;

            // Calculate h-cost (estimated cost to goal)
            double hCost = heuristic.estimate(candidate, goal, constraints);

            // Create and add the neighbor node
            RoutingNode neighbor = new RoutingNode(candidate, current, gCost, hCost);
            neighbors.add(neighbor);
        }

        return neighbors;
    }

    /**
     * Calculate the cost of a path segment
     */
    private double calculateSegmentCost(SpatialPoint from, SpatialPoint to, Set<RoutingConstraint> constraints) {
        // Base cost is the distance
        double cost = from.getCoordinate().distanceTo(to.getCoordinate());

        // Add penalties for constraint violations
        for (RoutingConstraint constraint : constraints) {
            if (!constraint.isHardConstraint() && constraint.isViolated(from, to)) {
                cost += constraint.getPenalty();
            }
        }

        return cost;
    }
}