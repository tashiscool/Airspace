package org.tash.trajectory;

import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.core.ElementType;
import org.tash.core.SpatialVisitor;
import org.tash.core.Visitor;
import org.tash.data.BoundingBox;
import org.tash.spatial.SpatialPoint;

import java.time.ZonedDateTime;

/**
 * Abstract base class for trajectory segments
 */
@Data
@SuperBuilder
@NoArgsConstructor
@AllArgsConstructor
@EqualsAndHashCode(onlyExplicitlyIncluded = true)
public abstract class AbstractTrajectorySegment implements TrajectorySegment {
    @EqualsAndHashCode.Include
    protected String id;
    protected SpatialPoint source;
    protected SpatialPoint target;
    protected ZonedDateTime startTime;
    protected ZonedDateTime endTime;
    protected TrajectoryType type;

    @Override
    public ElementType getElementType() {
        return ElementType.TRAJECTORY;
    }

    @Override
    public void accept(Visitor visitor) {
        if (visitor instanceof SpatialVisitor) {
            ((SpatialVisitor) visitor).visit(this);
        } else {
            visitor.visit(this);
        }
    }

    @Override
    public void accept(SpatialVisitor visitor) {
        visitor.visit(this);
    }

    @Override
    public BoundingBox getBoundingBox() {
        // Get base bounding box from the spatial line
        BoundingBox baseBoundingBox = toSpatialLine().getBoundingBox();

        // Add time dimension
        return BoundingBox.builder()
            .minLat(baseBoundingBox.getMinLat())
            .maxLat(baseBoundingBox.getMaxLat())
            .minLon(baseBoundingBox.getMinLon())
            .maxLon(baseBoundingBox.getMaxLon())
            .minAlt(baseBoundingBox.getMinAlt())
            .maxAlt(baseBoundingBox.getMaxAlt())
            .startTime(startTime)
            .endTime(endTime)
            .build();
    }
}