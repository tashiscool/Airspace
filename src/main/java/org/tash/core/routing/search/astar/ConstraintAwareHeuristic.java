package org.tash.core.routing.search.astar;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;

import java.util.Set;

/**
     * Heuristic that considers constraints
     */
    public class ConstraintAwareHeuristic implements HeuristicFunction {
        private final double constraintWeight;
        
        public ConstraintAwareHeuristic(double constraintWeight) {
            this.constraintWeight = constraintWeight;
        }
        
        @Override
        public double estimate(SpatialPoint current, SpatialPoint goal, Set<RoutingConstraint> constraints) {
            double baseEstimate = current.getCoordinate().distanceTo(goal.getCoordinate());
            
            // Create a direct line from current to goal
            SpatialLine directLine = SpatialLine.builder()
                .id("heuristic-line")
                .startPoint(current)
                .endPoint(goal)
                .build();
            
            // Check for constraint violations along the direct path
            double penalty = 0;
            for (RoutingConstraint constraint : constraints) {
                if (constraint.isViolated(current, goal)) {
                    if (constraint.isHardConstraint()) {
                        // Add a significant penalty for hard constraints
                        penalty += baseEstimate * constraintWeight;
                    } else {
                        // Add the configured penalty
                        penalty += constraint.getPenalty();
                    }
                }
            }
            
            return baseEstimate + penalty;
        }
    }