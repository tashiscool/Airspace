package org.tash.core.routing.raw;

import org.tash.core.SpatialElement;

/**
 * Abstract base class for routing constraints
 */
public abstract class AbstractRoutingConstraint implements RoutingConstraint {
    private final double penalty;
    private final boolean isHardConstraint;

    public AbstractRoutingConstraint(double penalty, boolean isHardConstraint) {
        this.penalty = penalty;
        this.isHardConstraint = isHardConstraint;
    }

    @Override
    public double getPenalty() {
        return penalty;
    }

    @Override
    public boolean isHardConstraint() {
        return isHardConstraint;
    }

    @Override
    public SpatialElement getSpatialElement() {
        return null; // Default implementation
    }
}
