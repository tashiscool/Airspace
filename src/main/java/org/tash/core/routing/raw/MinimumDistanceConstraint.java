package org.tash.core.routing.raw;

import org.tash.core.SpatialElement;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;

/**
 * Constraint for keeping minimum distance from a point
 */
public class MinimumDistanceConstraint extends AbstractRoutingConstraint {
    private final SpatialPoint referencePoint;
    private final double minimumDistanceNM;

    public MinimumDistanceConstraint(SpatialPoint referencePoint, double minimumDistanceNM,
                                     double penalty, boolean isHardConstraint) {
        super(penalty, isHardConstraint);
        this.referencePoint = referencePoint;
        this.minimumDistanceNM = minimumDistanceNM;
    }

    @Override
    public boolean isViolated(SpatialPoint from, SpatialPoint to) {
        // Create a temporary line segment
        SpatialLine segment = SpatialLine.builder()
                .id("temp-segment")
                .startPoint(from)
                .endPoint(to)
                .build();

        // Calculate minimum distance from the line segment to the reference point
        // This is an approximation using the Cross-Track Distance method
        double crossTrackDistance = Math.abs(referencePoint.getCoordinate()
                .crossTrackDistanceToPath(from.getCoordinate(), to.getCoordinate()));

        return crossTrackDistance < minimumDistanceNM;
    }

    @Override
    public SpatialElement getSpatialElement() {
        return referencePoint;
    }
}
