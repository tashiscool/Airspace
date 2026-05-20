package org.tash.trajectory;

import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.core.SpatialElement;
import org.tash.core.TemporalElement;
import org.tash.data.GeoCoordinate;
import org.tash.data.Vector3D;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.time.LinearMotionModel;

import java.time.ZonedDateTime;
import java.util.Date;

/**
     * Linear trajectory segment implementation
     */
    @Data
    @EqualsAndHashCode(callSuper = true)
    @SuperBuilder
    @NoArgsConstructor
    @AllArgsConstructor
    public class LinearTrajectorySegment extends AbstractTrajectorySegment {
        private LinearMotionModel motionModel;

    /**
         * Initialize motion model if not provided
         */
        public LinearMotionModel getMotionModel() {
            if (motionModel == null) {
                motionModel = LinearMotionModel.builder()
                    .startPosition(source.getCoordinate())
                    .endPosition(target.getCoordinate())
                    .startTime(startTime)
                    .endTime(endTime)
                    .build();
            }
            return motionModel;
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
        public SpatialPoint getPointAtFraction(double fraction) {
            // Clamp fraction between 0 and 1
            fraction = Math.max(0, Math.min(1, fraction));
            
            // Interpolate between start and end
            GeoCoordinate interpolated = source.getCoordinate()
                .interpolate(target.getCoordinate(), fraction);
            
            return SpatialPoint.builder()
                .id(id + "-point-" + fraction)
                .coordinate(interpolated)
                .build();
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
        return Vector3D.fromGeoCoordinate(source.getCoordinate());
    }

    @Override
    public Vector3D getEndPosition() {
        return Vector3D.fromGeoCoordinate(target.getCoordinate());
    }

    @Override
    public boolean intersectsVolume(SpatialVolume volume) {
        if (volume == null) {
            return false;
        }
        for (int i = 0; i <= 100; i++) {
            SpatialPoint point = getPointAtFraction(i / 100.0);
            if (volume.contains(point)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public Date getDuration() {
        return new Date(java.time.Duration.between(startTime, endTime).toMillis());
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
