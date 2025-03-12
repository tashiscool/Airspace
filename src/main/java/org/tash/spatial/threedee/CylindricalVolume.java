package org.tash.spatial.threedee;

import lombok.*;
import org.tash.core.*;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialCircle;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.time.ZonedDateTime;

/**
 * Represents a cylindrical volume in 3D space
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
@EqualsAndHashCode(onlyExplicitlyIncluded = true)
public class CylindricalVolume implements SpatialElement, TemporalElement {
    @EqualsAndHashCode.Include
    private String id;
    private SpatialCircle base;
    private double lowerAltitude;
    private double upperAltitude;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;

    public CylindricalVolume(String id, GeoCoordinate geoCoordinate, double minAltitude, double maxAltitude, ZonedDateTime startTime, ZonedDateTime endTime) {
        this.id = id;
        this.base = SpatialCircle.builder()
            .id(id + "-base")
            .center(SpatialPoint.builder()
                .id(id + "-center")
                .coordinate(geoCoordinate)
                .build())
            .radius(0.0)
            .build();
        this.lowerAltitude = minAltitude;
        this.upperAltitude = maxAltitude;
        this.startTime = startTime;
        this.endTime = endTime;
    }

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
        BoundingBox baseBoundingBox = base.getBoundingBox();
        
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
        if (v < 0.0 || v > 1.0) {
            throw new IllegalArgumentException("Fraction must be between 0.0 and 1.0");
        }

        double altitude = lowerAltitude + v * (upperAltitude - lowerAltitude);
        GeoCoordinate baseCenter = base.getCenter().getCoordinate();

        return SpatialPoint.builder()
                .id(id + "-point-" + v)
                .coordinate(GeoCoordinate.builder()
                        .latitude(baseCenter.getLatitude())
                        .longitude(baseCenter.getLongitude())
                        .altitude(altitude)
                        .build())
                .build();
    }

    /**
     * Check if the volume contains a point
     */
    public boolean containsPoint(GeoCoordinate point) {
        // Check altitude bounds
        if (point.getAltitude() < lowerAltitude || point.getAltitude() > upperAltitude) {
            return false;
        }
        
        // Check if the point is within the base circle
        return base.containsPoint(point);
    }
    
    /**
     * Convert to a polygon-based volume for compatibility
     */
    public SpatialVolume toPolygonalVolume(int numPoints) {
        SpatialPolygon basePolygon = base.toPolygon(numPoints);
        
        return SpatialVolume.builder()
            .id(id + "-poly")
            .basePolygon(basePolygon)
            .lowerAltitude(lowerAltitude)
            .upperAltitude(upperAltitude)
            .startTime(startTime)
            .endTime(endTime)
            .build();
    }

    @Override
    public boolean timeOverlaps(TemporalElement other) {
        return TemporalElement.super.timeOverlaps(other);
    }

    @Override
    public boolean containsTime(ZonedDateTime time) {
        return TemporalElement.super.containsTime(time);
    }
}