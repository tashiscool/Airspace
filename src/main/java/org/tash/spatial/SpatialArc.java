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

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
@EqualsAndHashCode(onlyExplicitlyIncluded = true)
public class SpatialArc implements SpatialElement {
    @EqualsAndHashCode.Include
    private String id;
    private SpatialPoint center;
    private double radius; // in nautical miles
    private double startAngle; // in degrees
    private double endAngle; // in degrees
    private boolean isClockwise; // direction of arc
    
    @Override
    public ElementType getElementType() {
        return ElementType.ARC;
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
        // Normalize angles to 0-360 range
        double normStart = normalizeAngle(startAngle);
        double normEnd = normalizeAngle(endAngle);
        
        // Calculate bounding box by sampling points along the arc
        List<SpatialPoint> samplePoints = getSamplePoints(12);
        
        // Start with the center point's coordinates
        GeoCoordinate centerCoord = center.getCoordinate();
        double minLat = centerCoord.getLatitude();
        double maxLat = centerCoord.getLatitude();
        double minLon = centerCoord.getLongitude();
        double maxLon = centerCoord.getLongitude();
        double alt = centerCoord.getAltitude();
        
        // Also include arc endpoints
        SpatialPoint startPoint = getPointAtAngle(startAngle);
        SpatialPoint endPoint = getPointAtAngle(endAngle);
        
        samplePoints.add(startPoint);
        samplePoints.add(endPoint);
        
        // Find min/max coordinates from sample points
        for (SpatialPoint point : samplePoints) {
            GeoCoordinate coord = point.getCoordinate();
            minLat = Math.min(minLat, coord.getLatitude());
            maxLat = Math.max(maxLat, coord.getLatitude());
            minLon = Math.min(minLon, coord.getLongitude());
            maxLon = Math.max(maxLon, coord.getLongitude());
        }
        
        // Add padding
        double latPadding = (maxLat - minLat) * 0.1 + 0.001;
        double lonPadding = (maxLon - minLon) * 0.1 + 0.001;
        
        return BoundingBox.builder()
            .minLat(minLat - latPadding)
            .maxLat(maxLat + latPadding)
            .minLon(minLon - lonPadding)
            .maxLon(maxLon + lonPadding)
            .minAlt(alt - 10)
            .maxAlt(alt + 10)
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
        double angle = startAngle + v * (endAngle - startAngle);
        return getPointAtAngle(angle);
    }

    /**
     * Normalize angle to 0-360 range
     */
    public double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }
    
    /**
     * Get a point on the arc at a specific angle
     * @param angle in degrees
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
     * Check if a given angle is within the arc's sweep
     */
    public boolean containsAngle(double angle) {
        double normAngle = normalizeAngle(angle);
        double normStart = normalizeAngle(startAngle);
        double normEnd = normalizeAngle(endAngle);
        
        if (isClockwise) {
            if (normStart > normEnd) {
                return normAngle >= normEnd && normAngle <= normStart;
            } else {
                return normAngle >= normEnd || normAngle <= normStart;
            }
        } else {
            if (normStart < normEnd) {
                return normAngle >= normStart && normAngle <= normEnd;
            } else {
                return normAngle >= normStart || normAngle <= normEnd;
            }
        }
    }
    
    /**
     * Check if the arc contains a point
     */
    public boolean containsPoint(GeoCoordinate point) {
        // First check distance from center is approximately the radius
        double distance = center.getCoordinate().distanceTo(point);
        
        // Allow for some tolerance in the radius check
        double tolerance = radius * 0.05; // 5% tolerance
        if (Math.abs(distance - radius) > tolerance) {
            return false;
        }
        
        // Calculate the angle of the point
        GeoCoordinate centerCoord = center.getCoordinate();
        double dx = point.getLongitude() - centerCoord.getLongitude();
        double dy = point.getLatitude() - centerCoord.getLatitude();
        
        // Adjust for the fact that longitude degrees vary with latitude
        dx = dx * Math.cos(Math.toRadians(centerCoord.getLatitude()));
        
        double angle = Math.toDegrees(Math.atan2(dy, dx));
        if (angle < 0) angle += 360;
        
        // Check if the angle is within the arc's sweep
        return containsAngle(angle);
    }
    
    /**
     * Get sample points along the arc
     */
    public List<SpatialPoint> getSamplePoints(int numPoints) {
        List<SpatialPoint> points = new ArrayList<>();
        
        double normStart = normalizeAngle(startAngle);
        double normEnd = normalizeAngle(endAngle);
        
        // Calculate sweep angle with consideration for direction
        double sweep;
        if (isClockwise) {
            sweep = normStart <= normEnd ? 
                360 - (normEnd - normStart) : normStart - normEnd;
        } else {
            sweep = normStart <= normEnd ? 
                normEnd - normStart : 360 - (normStart - normEnd);
        }
        
        // Generate points
        for (int i = 0; i <= numPoints; i++) {
            double fraction = (double) i / numPoints;
            double angleOffset = isClockwise ? -fraction * sweep : fraction * sweep;
            double angle = normalizeAngle(normStart + angleOffset);
            
            points.add(getPointAtAngle(angle));
        }
        
        return points;
    }
    
    /**
     * Convert the arc to a polyline approximation
     */
    public SpatialLine toPolyline(int numSegments) {
        List<SpatialPoint> points = getSamplePoints(numSegments);
        
        if (points.size() < 2) {
            throw new IllegalStateException("Not enough points to create a polyline");
        }
        
        // Create connected line segments
        List<SpatialLine> segments = new ArrayList<>();
        for (int i = 0; i < points.size() - 1; i++) {
            segments.add(SpatialLine.builder()
                .id(id + "-segment-" + i)
                .startPoint(points.get(i))
                .endPoint(points.get(i + 1))
                .build());
        }
        
        // For simplicity, return just the first and last points to form a single line
        return SpatialLine.builder()
            .id(id + "-line")
            .startPoint(points.get(0))
            .endPoint(points.get(points.size() - 1))
            .build();
    }
}