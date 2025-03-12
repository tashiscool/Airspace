package org.tash.core.routing.raw;

import org.tash.spatial.SpatialPoint;

import java.util.List;
import java.util.Set;

/**
 * Result of a route planning operation
 */
public class RoutingResult {
    private final List<SpatialPoint> waypoints;
    private final double totalDistance;
    private final double totalCost;
    private final Set<RoutingConstraint> violatedConstraints;
    private final boolean isSuccessful;

    public RoutingResult(List<SpatialPoint> waypoints, double totalDistance,
                         double totalCost, Set<RoutingConstraint> violatedConstraints) {
        this.waypoints = waypoints;
        this.totalDistance = totalDistance;
        this.totalCost = totalCost;
        this.violatedConstraints = violatedConstraints;
        this.isSuccessful = !waypoints.isEmpty();
    }

    public List<SpatialPoint> getWaypoints() {
        return waypoints;
    }

    public double getTotalDistance() {
        return totalDistance;
    }

    public double getTotalCost() {
        return totalCost;
    }

    public Set<RoutingConstraint> getViolatedConstraints() {
        return violatedConstraints;
    }

    public boolean isSuccessful() {
        return isSuccessful;
    }
}
