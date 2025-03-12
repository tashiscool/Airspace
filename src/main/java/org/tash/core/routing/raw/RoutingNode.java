package org.tash.core.routing.raw;

import org.tash.spatial.SpatialPoint;

/**
 * Node used in path-finding algorithms
 */
public class RoutingNode implements Comparable<RoutingNode> {
    private final SpatialPoint point;
    private final RoutingNode parent;
    private final double gCost; // Cost from start
    private final double hCost; // Heuristic cost to goal
    private final double fCost; // Total cost (g + h)

    public RoutingNode(SpatialPoint point, RoutingNode parent, double gCost, double hCost) {
        this.point = point;
        this.parent = parent;
        this.gCost = gCost;
        this.hCost = hCost;
        this.fCost = gCost + hCost;
    }

    public SpatialPoint getPoint() {
        return point;
    }

    public RoutingNode getParent() {
        return parent;
    }

    public double getGCost() {
        return gCost;
    }

    public double getHCost() {
        return hCost;
    }

    public double getFCost() {
        return fCost;
    }

    @Override
    public int compareTo(RoutingNode other) {
        // Compare by f-cost for priority queue
        return Double.compare(this.fCost, other.fCost);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;

        RoutingNode other = (RoutingNode) obj;
        return point.equals(other.point);
    }

    @Override
    public int hashCode() {
        return point.hashCode();
    }
}
