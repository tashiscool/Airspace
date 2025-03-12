package org.tash.spatial;

import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.core.*;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

/**
     * Represents a volume in space (3D polygon with height)
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    @EqualsAndHashCode(onlyExplicitlyIncluded = true)
    public class SpatialVolume implements SpatialElement, TemporalElement {
        @EqualsAndHashCode.Include
        private String id;
        private SpatialPolygon basePolygon;
        private double lowerAltitude;
        private double upperAltitude;
        private ZonedDateTime startTime;
        private ZonedDateTime endTime;
        
        @Override
        public ElementType getElementType() {
            return ElementType.VOLUME;
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
            BoundingBox baseBoundingBox = basePolygon.getBoundingBox();
            
            return BoundingBox.builder()
                .minLat(baseBoundingBox.getMinLat())
                .maxLat(baseBoundingBox.getMaxLat())
                .minLon(baseBoundingBox.getMinLon())
                .maxLon(baseBoundingBox.getMaxLon())
                .minAlt(lowerAltitude)
                .maxAlt(upperAltitude)
                .startTime(startTime)
                .endTime(endTime)
                .build();
        }

    @Override
    public boolean intersects(SpatialElement other) {
        return SpatialElement.super.intersects(other);
    }

    @Override
    public boolean contains(double lat, double lon, double alt) {
        return SpatialElement.super.contains(lat, lon, alt);
    }

    @Override
    public SpatialPoint getPointAtFraction(double v) {
            throw new UnsupportedOperationException("Operation not supported for a volume.");
    }

    /**
         * Check if the volume contains a point
         */
        public boolean containsPoint(GeoCoordinate point) {
            // Check altitude bounds
            if (point.getAltitude() < lowerAltitude || point.getAltitude() > upperAltitude) {
                return false;
            }
            
            // Check if the point is within the base polygon
            return basePolygon.containsPoint(point);
        }

    @Override
    public boolean timeOverlaps(TemporalElement other) {
        return TemporalElement.super.timeOverlaps(other);
    }

    @Override
    public boolean containsTime(ZonedDateTime time) {
        return TemporalElement.super.containsTime(time);
    }

    public boolean contains(SpatialPoint point) {
            return containsPoint(point.getCoordinate());
    }

    public boolean contains(GeoCoordinate point) {
            return containsPoint(point);
    }
}