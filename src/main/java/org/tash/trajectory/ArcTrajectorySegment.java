package org.tash.trajectory;

import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.core.SpatialElement;
import org.tash.core.TemporalElement;
import org.tash.data.BoundingBox;
import org.tash.data.Vector3D;
import org.tash.spatial.SpatialArc;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.time.ArcMotionModel;

import java.time.ZonedDateTime;
import java.util.Date;


/**
 * Represents a curved trajectory using an arc
 */
@Data
@EqualsAndHashCode(callSuper = true)
@SuperBuilder
@NoArgsConstructor
@AllArgsConstructor
public class ArcTrajectorySegment extends AbstractTrajectorySegment {
    private SpatialArc arc;
    private ArcMotionModel motionModel;
    
    /**
     * Initialize motion model if not provided
     */
    public ArcMotionModel getMotionModel() {
        if (motionModel == null) {
            motionModel = ArcMotionModel.builder()
                .center(arc.getCenter().getCoordinate())
                .radius(arc.getRadius())
                .startAngle(arc.getStartAngle())
                .endAngle(arc.getEndAngle())
                .isClockwise(arc.isClockwise())
                .startTime(startTime)
                .endTime(endTime)
                .build();
        }
        return motionModel;
    }
    
    @Override
    public SpatialPoint getPointAtFraction(double fraction) {
        double normStart = arc.normalizeAngle(arc.getStartAngle());
        double normEnd = arc.normalizeAngle(arc.getEndAngle());
        
        // Calculate sweep angle with consideration for direction
        double sweep;
        if (arc.isClockwise()) {
            sweep = normStart <= normEnd ? 
                360 - (normEnd - normStart) : normStart - normEnd;
        } else {
            sweep = normStart <= normEnd ? 
                normEnd - normStart : 360 - (normStart - normEnd);
        }
        
        // Calculate angle at fraction
        double angleOffset = arc.isClockwise() ? -fraction * sweep : fraction * sweep;
        double angle = arc.normalizeAngle(normStart + angleOffset);
        
        // Get point at that angle
        return arc.getPointAtAngle(angle);
    }

    @Override
    public SpatialPoint getPointAtTime(ZonedDateTime time) {
        return super.getPointAtTime(time);
    }

    @Override
    public SpatialLine toSpatialLine() {
        return super.toSpatialLine();
    }

    @Override
    public Vector3D getStartPosition() {
        return null;
    }

    @Override
    public Vector3D getEndPosition() {
        return null;
    }

    @Override
    public boolean intersectsVolume(SpatialVolume volume) {
        return false;
    }

    @Override
    public Date getDuration() {
        return null;
    }

    @Override
    public BoundingBox getBoundingBox() {
        // Get base bounding box from the arc
        BoundingBox arcBoundingBox = arc.getBoundingBox();
        
        // Add time dimension
        return BoundingBox.builder()
            .minLat(arcBoundingBox.getMinLat())
            .maxLat(arcBoundingBox.getMaxLat())
            .minLon(arcBoundingBox.getMinLon())
            .maxLon(arcBoundingBox.getMaxLon())
            .minAlt(arcBoundingBox.getMinAlt())
            .maxAlt(arcBoundingBox.getMaxAlt())
            .startTime(startTime)
            .endTime(endTime)
            .build();
    }

    @Override
    public boolean intersects(SpatialElement other) {
        return super.intersects(other);
    }

    @Override
    public boolean contains(double lat, double lon, double alt) {
        return super.contains(lat, lon, alt);
    }

    @Override
    public boolean timeOverlaps(TemporalElement other) {
        return super.timeOverlaps(other);
    }

    @Override
    public boolean containsTime(ZonedDateTime time) {
        return super.containsTime(time);
    }
}