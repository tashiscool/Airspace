package org.tash.core.routing.raw;

import org.tash.spatial.SpatialPoint;

/**
 * Constraint for maximum climb or descent rate
 */
public class ClimbDescentRateConstraint extends AbstractRoutingConstraint {
    private final double maxClimbRateFtPerNM;
    private final double maxDescentRateFtPerNM;

    public ClimbDescentRateConstraint(double maxClimbRateFtPerNM, double maxDescentRateFtPerNM,
                                      double penalty, boolean isHardConstraint) {
        super(penalty, isHardConstraint);
        this.maxClimbRateFtPerNM = maxClimbRateFtPerNM;
        this.maxDescentRateFtPerNM = maxDescentRateFtPerNM;
    }

    @Override
    public boolean isViolated(SpatialPoint from, SpatialPoint to) {
        double fromAlt = from.getCoordinate().getAltitude();
        double toAlt = to.getCoordinate().getAltitude();
        double altDiff = toAlt - fromAlt;

        // Calculate distance between points
        double distanceNM = from.getCoordinate().distanceTo(to.getCoordinate());

        // Avoid division by zero
        if (distanceNM < 0.1) {
            return false;
        }

        // Calculate climb/descent rate
        double ratePerNM = altDiff / distanceNM;

        // Check if rate exceeds limits
        return (ratePerNM > 0 && ratePerNM > maxClimbRateFtPerNM) ||
                (ratePerNM < 0 && -ratePerNM > maxDescentRateFtPerNM);
    }
}
