package org.tash.core.routing.search.astar;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.spatial.SpatialPoint;

import java.util.Set;

/**
 * Heuristic that considers altitude changes
 */
public class AltitudeAwareHeuristic implements HeuristicFunction {
    private final double weightAltitude;

    public AltitudeAwareHeuristic(double weightAltitude) {
        this.weightAltitude = weightAltitude;
    }

    @Override
    public double estimate(SpatialPoint current, SpatialPoint goal, Set<RoutingConstraint> constraints) {
        double horizontalDistance = current.getCoordinate().distanceTo(goal.getCoordinate());

        // Get altitude difference in thousands of feet
        double altDiff = Math.abs(current.getCoordinate().getAltitude() - goal.getCoordinate().getAltitude()) / 1000.0;

        // Combine horizontal and vertical components
        return horizontalDistance + altDiff * weightAltitude;
    }
}