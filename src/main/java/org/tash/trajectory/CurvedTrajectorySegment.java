package org.tash.trajectory;

import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.core.SpatialElement;
import org.tash.core.TemporalElement;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import org.tash.data.Vector3D;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.time.CurvedMotionModel;

import java.time.ZonedDateTime;
import java.util.Date;

/**
     * Curved trajectory segment implementation
     */
    @Data
    @EqualsAndHashCode(callSuper = true)
    @SuperBuilder
    @NoArgsConstructor
    @AllArgsConstructor
    public class CurvedTrajectorySegment extends AbstractTrajectorySegment {
        private SpatialPoint controlPoint;
        private CurvedMotionModel motionModel;
        
        /**
         * Initialize motion model if not provided
         */
        public CurvedMotionModel getMotionModel() {
            if (motionModel == null) {
                motionModel = CurvedMotionModel.builder()
                    .startPosition(source.getCoordinate())
                    .endPosition(target.getCoordinate())
                    .controlPoint(controlPoint.getCoordinate())
                    .startTime(startTime)
                    .endTime(endTime)
                    .build();
            }
            return motionModel;
        }
        
        @Override
        public SpatialPoint getPointAtFraction(double fraction) {
            // Bezier curve interpolation
            double t = Math.max(0, Math.min(1, fraction));
            double mt = 1 - t;
            double mt2 = mt * mt;
            double t2 = t * t;
            
            GeoCoordinate p0 = source.getCoordinate();
            GeoCoordinate p1 = controlPoint.getCoordinate();
            GeoCoordinate p2 = target.getCoordinate();
            
            double lat = mt2 * p0.getLatitude() + 
                        2 * mt * t * p1.getLatitude() + 
                        t2 * p2.getLatitude();
                        
            double lon = mt2 * p0.getLongitude() + 
                        2 * mt * t * p1.getLongitude() + 
                        t2 * p2.getLongitude();
                        
            double alt = mt2 * p0.getAltitude() + 
                        2 * mt * t * p1.getAltitude() + 
                        t2 * p2.getAltitude();
            
            return SpatialPoint.builder()
                .id(id + "-point-" + fraction)
                .coordinate(GeoCoordinate.builder()
                    .latitude(lat)
                    .longitude(lon)
                    .altitude(alt)
                    .build())
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
            // For curved segments, consider the control point as well
            BoundingBox baseBoundingBox = super.getBoundingBox();
            BoundingBox controlBoundingBox = controlPoint.getBoundingBox();
            
            return baseBoundingBox.merge(controlBoundingBox);
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