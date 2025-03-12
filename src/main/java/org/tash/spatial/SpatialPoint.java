package org.tash.spatial;

import lombok.*;
import org.tash.core.ElementType;
import org.tash.core.SpatialElement;
import org.tash.core.SpatialVisitor;
import org.tash.core.Visitor;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;

/**
 * Represents a point in space
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
@EqualsAndHashCode(onlyExplicitlyIncluded = true)
public class SpatialPoint implements SpatialElement {
    @EqualsAndHashCode.Include
    private String id;
    private GeoCoordinate coordinate;

    public SpatialPoint(GeoCoordinate geoCoordinate) {
        this.coordinate = geoCoordinate;
    }

    @Override
    public ElementType getElementType() {
        return ElementType.POINT;
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
        // Small bounding box around the point
        double padding = 0.00001; // Small padding
        return BoundingBox.builder()
                .minLat(coordinate.getLatitude() - padding)
                .maxLat(coordinate.getLatitude() + padding)
                .minLon(coordinate.getLongitude() - padding)
                .maxLon(coordinate.getLongitude() + padding)
                .minAlt(coordinate.getAltitude() - 1)
                .maxAlt(coordinate.getAltitude() + 1)
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
        throw new UnsupportedOperationException("Operation not supported for a single point.");
    }

    /**
     * Calculate distance to another point
     */
    public double distanceTo(SpatialPoint other) {
        return coordinate.distanceTo(other.coordinate);
    }
}