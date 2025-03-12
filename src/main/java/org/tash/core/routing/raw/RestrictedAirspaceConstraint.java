package org.tash.core.routing.raw;

import org.tash.core.SpatialElement;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;

/**
 * Constraint for avoiding a restricted airspace
 */
public class RestrictedAirspaceConstraint extends AbstractRoutingConstraint {
    private final SpatialElement restrictedArea;

    public RestrictedAirspaceConstraint(SpatialElement restrictedArea, double penalty, boolean isHardConstraint) {
        super(penalty, isHardConstraint);
        this.restrictedArea = restrictedArea;
    }

    @Override
    public boolean isViolated(SpatialPoint from, SpatialPoint to) {
        // Create a temporary line segment and check if it intersects the restricted area
        SpatialLine segment = SpatialLine.builder()
                .id("temp-segment")
                .startPoint(from)
                .endPoint(to)
                .build();

        return segment.intersects(restrictedArea);
    }

    @Override
    public SpatialElement getSpatialElement() {
        return restrictedArea;
    }
}
