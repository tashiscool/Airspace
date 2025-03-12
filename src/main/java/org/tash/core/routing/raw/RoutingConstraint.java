package org.tash.core.routing.raw;

import org.tash.core.SpatialElement;
import org.tash.spatial.SpatialPoint;

/**
 * Interface for routing constraints
 */
public interface RoutingConstraint {
    /**
     * Check if a path segment violates this constraint
     *
     * @param from Starting point
     * @param to   Ending point
     * @return True if the segment violates the constraint
     */
    boolean isViolated(SpatialPoint from, SpatialPoint to);

    /**
     * Get the cost penalty for violating this constraint
     *
     * @return Cost penalty value
     */
    double getPenalty();

    /**
     * Get the spatial element associated with this constraint, if any
     *
     * @return Spatial element or null if not applicable
     */
    SpatialElement getSpatialElement();

    /**
     * Check if this is a hard constraint that cannot be violated
     *
     * @return True if this is a hard constraint
     */
    boolean isHardConstraint();
}
