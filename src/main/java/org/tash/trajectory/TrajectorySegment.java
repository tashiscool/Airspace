package org.tash.trajectory;

import org.tash.core.SpatialElement;
import org.tash.core.TemporalElement;
import org.tash.data.GeoCoordinate;
import org.tash.data.Vector3D;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.time.MotionModel;

import java.time.ZonedDateTime;
import java.util.Date;

/**
     * Interface for trajectory segments
     */
    public interface TrajectorySegment extends SpatialElement, TemporalElement {
        /**
         * Get the source point
         */
        SpatialPoint getSource();
        
        /**
         * Get the target point
         */
        SpatialPoint getTarget();
        
        /**
         * Get the trajectory type
         */
        TrajectoryType getType();
        
        /**
         * Get the motion model for this segment
         */
        MotionModel getMotionModel();
        
        /**
         * Get a point at a specific fraction along the segment
         */
        SpatialPoint getPointAtFraction(double fraction);
        
        /**
         * Get a point at a specific time
         */
        default SpatialPoint getPointAtTime(ZonedDateTime time) {
            GeoCoordinate coord = getMotionModel().getPositionAt(time);
            return SpatialPoint.builder()
                .id(getId() + "-at-" + time)
                .coordinate(coord)
                .build();
        }
        
        /**
         * Convert to a spatial line (for simplified spatial operations)
         */
        default SpatialLine toSpatialLine() {
            return SpatialLine.builder()
                .id(getId() + "-line")
                .startPoint(getSource())
                .endPoint(getTarget())
                .build();
        }

    Vector3D getStartPosition();

    Vector3D getEndPosition();

    boolean intersectsVolume(SpatialVolume volume);

    Date getDuration();
}