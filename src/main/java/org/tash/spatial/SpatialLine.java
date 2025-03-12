package org.tash.spatial;

import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.core.ElementType;
import org.tash.core.SpatialElement;
import org.tash.core.SpatialVisitor;
import org.tash.core.Visitor;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;

/**
     * Represents a line in space
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    @EqualsAndHashCode(onlyExplicitlyIncluded = true)
    public class SpatialLine implements SpatialElement {
    @EqualsAndHashCode.Include
    private String id;
    private SpatialPoint startPoint;
    private SpatialPoint endPoint;

    public SpatialLine(SpatialPoint spatialPoint, SpatialPoint spatialPoint1) {
        this.startPoint = spatialPoint;
        this.endPoint = spatialPoint1;
    }

    @Override
    public ElementType getElementType() {
        return ElementType.LINE;
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
        GeoCoordinate start = startPoint.getCoordinate();
        GeoCoordinate end = endPoint.getCoordinate();

        double minLat = Math.min(start.getLatitude(), end.getLatitude());
        double maxLat = Math.max(start.getLatitude(), end.getLatitude());
        double minLon = Math.min(start.getLongitude(), end.getLongitude());
        double maxLon = Math.max(start.getLongitude(), end.getLongitude());
        double minAlt = Math.min(start.getAltitude(), end.getAltitude());
        double maxAlt = Math.max(start.getAltitude(), end.getAltitude());

        // Add padding
        double latPadding = (maxLat - minLat) * 0.01 + 0.0001;
        double lonPadding = (maxLon - minLon) * 0.01 + 0.0001;
        double altPadding = (maxAlt - minAlt) * 0.01 + 10;
        return BoundingBox.builder()
                .minLat(minLat - latPadding)
                .maxLat(maxLat + latPadding)
                .minLon(minLon - lonPadding)
                .maxLon(maxLon + lonPadding)
                .minAlt(minAlt - altPadding)
                .maxAlt(maxAlt + altPadding)
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
        double lat = startPoint.getCoordinate().getLatitude() + v * (endPoint.getCoordinate().getLatitude() - startPoint.getCoordinate().getLatitude());
        double lon = startPoint.getCoordinate().getLongitude() + v * (endPoint.getCoordinate().getLongitude() - startPoint.getCoordinate().getLongitude());
        double alt = startPoint.getCoordinate().getAltitude() + v * (endPoint.getCoordinate().getAltitude() - startPoint.getCoordinate().getAltitude());
        return SpatialPoint.builder()
                .id(id)
                .coordinate(GeoCoordinate.builder()
                        .latitude(lat)
                        .longitude(lon)
                        .altitude(alt)
                        .build())
                .build();
    }

    public boolean containsPoint(GeoCoordinate coordinate) {
        GeoCoordinate start = startPoint.getCoordinate();
        GeoCoordinate end = endPoint.getCoordinate();

        // Check if the point is within the bounding box
        BoundingBox bbox = getBoundingBox();
        if (!bbox.contains(coordinate)) {
            return false;
        }

        // Check if the point is on the line
        double dxc = coordinate.getLongitude() - start.getLongitude();
        double dyc = coordinate.getLatitude() - start.getLatitude();
        double dxl = end.getLongitude() - start.getLongitude();
        double dyl = end.getLatitude() - start.getLatitude();
        double cross = dxc * dyl - dyc * dxl;
        if (Math.abs(cross) > 0.0001) {
            return false;
        }

        // Check if the point is within the line segment
        if (Math.abs(dxl) >= Math.abs(dyl)) {
            return dxl > 0 ? start.getLongitude() <= coordinate.getLongitude() && coordinate.getLongitude() <= end.getLongitude()
                    : end.getLongitude() <= coordinate.getLongitude() && coordinate.getLongitude() <= start.getLongitude();
        } else {
            return dyl > 0 ? start.getLatitude() <= coordinate.getLatitude() && coordinate.getLatitude() <= end.getLatitude()
                    : end.getLatitude() <= coordinate.getLatitude() && coordinate.getLatitude() <= start.getLatitude();
        }
    }
}
