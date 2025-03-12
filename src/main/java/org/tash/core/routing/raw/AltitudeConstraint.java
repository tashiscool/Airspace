package org.tash.core.routing.raw;

import org.tash.spatial.SpatialPoint;

/**
 * Constraint for minimum and maximum altitudes
 */
public class AltitudeConstraint extends AbstractRoutingConstraint {
    private final double minAltitude;
    private final double maxAltitude;

    public AltitudeConstraint(double minAltitude, double maxAltitude, double penalty, boolean isHardConstraint) {
        super(penalty, isHardConstraint);
        this.minAltitude = minAltitude;
        this.maxAltitude = maxAltitude;
    }

    @Override
    public boolean isViolated(SpatialPoint from, SpatialPoint to) {
        double fromAlt = from.getCoordinate().getAltitude();
        double toAlt = to.getCoordinate().getAltitude();

        // Check if either endpoint violates altitude constraints
        return fromAlt < minAltitude || fromAlt > maxAltitude ||
                toAlt < minAltitude || toAlt > maxAltitude;
    }
}
