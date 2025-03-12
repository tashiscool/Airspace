package org.tash.spatial;

import lombok.*;
import org.tash.core.ElementType;
import org.tash.core.SpatialElement;
import org.tash.core.SpatialVisitor;
import org.tash.core.Visitor;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a circle in space
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
@EqualsAndHashCode(onlyExplicitlyIncluded = true)
public class SpatialCircle implements SpatialElement {
    @EqualsAndHashCode.Include
    private String id;
    private SpatialPoint center;
    private double radius; // in nautical miles
    
    @Override
    public ElementType getElementType() {
        return ElementType.CIRCLE;
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
        GeoCoordinate centerCoord = center.getCoordinate();
        
        // Convert radius from nautical miles to degrees (approximate)
        // 1 minute of latitude = 1 nautical mile
        double latDelta = radius / 60.0;
        
        // Longitude degrees depend on latitude (they get farther apart near the equator)
        double lonDelta = radius / (60.0 * Math.cos(Math.toRadians(centerCoord.getLatitude())));
        
        return BoundingBox.builder()
            .minLat(centerCoord.getLatitude() - latDelta)
            .maxLat(centerCoord.getLatitude() + latDelta)
            .minLon(centerCoord.getLongitude() - lonDelta)
            .maxLon(centerCoord.getLongitude() + lonDelta)
            .minAlt(centerCoord.getAltitude() - 10) // Slight padding
            .maxAlt(centerCoord.getAltitude() + 10)
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
        SpatialPoint pt = getPointAtFraction(v);
        double lat = center.getCoordinate().getLatitude() + v * (pt.getCoordinate().getLatitude() - center.getCoordinate().getLatitude());
        double lon = center.getCoordinate().getLongitude() + v * (pt.getCoordinate().getLongitude() - center.getCoordinate().getLongitude());
        double angle = Math.toDegrees(2 * Math.asin(radius / center.getCoordinate().distanceTo(new GeoCoordinate(lat, lon, center.getCoordinate().getAltitude()))));
        return getPointAtAngle(angle).getCoordinate().equals(new GeoCoordinate(lat, lon, center.getCoordinate().getAltitude())) ? getPointAtAngle(angle) : getPointAtAngle(360 - angle);
    }

    /**
     * Check if a point is inside the circle
     */
    public boolean containsPoint(GeoCoordinate point) {
        double distance = center.getCoordinate().distanceTo(point);
        return distance <= radius;
    }
    
    /**
     * Get a point on the circumference of the circle at a specific angle
     * @param angle in degrees, 0 = east, 90 = north, etc.
     */
    public SpatialPoint getPointAtAngle(double angle) {
        double angleRad = Math.toRadians(angle);
        
        GeoCoordinate centerCoord = center.getCoordinate();
        double lat = centerCoord.getLatitude();
        double lon = centerCoord.getLongitude();
        
        // Convert radius from nautical miles to appropriate deltas
        double latDelta = radius * Math.sin(angleRad) / 60.0;
        double lonDelta = radius * Math.cos(angleRad) / (60.0 * Math.cos(Math.toRadians(lat)));
        
        return SpatialPoint.builder()
            .id(id + "-point-" + angle)
            .coordinate(GeoCoordinate.builder()
                .latitude(lat + latDelta)
                .longitude(lon + lonDelta)
                .altitude(centerCoord.getAltitude())
                .build())
            .build();
    }
    
    /**
     * Generate a polygon approximation of the circle
     * @param numPoints number of points to use for approximation
     */
    public SpatialPolygon toPolygon(int numPoints) {
        List<SpatialPoint> points = new ArrayList<>();
        
        for (int i = 0; i < numPoints; i++) {
            double angle = 360.0 * i / numPoints;
            points.add(getPointAtAngle(angle));
        }
        
        return SpatialPolygon.builder()
            .id(id + "-polygon")
            .vertices(points)
            .build();
    }
}