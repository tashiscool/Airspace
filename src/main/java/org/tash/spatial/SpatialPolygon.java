package org.tash.spatial;


import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.core.ElementType;
import org.tash.core.SpatialElement;
import org.tash.core.SpatialVisitor;
import org.tash.core.Visitor;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.List;

/**
     * Represents a polygon in space
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    @EqualsAndHashCode(onlyExplicitlyIncluded = true)
    public class SpatialPolygon implements SpatialElement {
        @EqualsAndHashCode.Include
        private String id;
        
        @Builder.Default
        private List<SpatialPoint> vertices = new ArrayList<>();

    public SpatialPolygon(List<SpatialPoint> boundaryPoints) {
        this.vertices = boundaryPoints;
    }

    @Override
        public ElementType getElementType() {
            return ElementType.POLYGON;
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
            if (vertices.isEmpty()) {
                throw new IllegalStateException("Polygon has no vertices");
            }
            
            double minLat = Double.MAX_VALUE;
            double maxLat = -Double.MAX_VALUE;
            double minLon = Double.MAX_VALUE;
            double maxLon = -Double.MAX_VALUE;
            double minAlt = Double.MAX_VALUE;
            double maxAlt = -Double.MAX_VALUE;
            
            for (SpatialPoint vertex : vertices) {
                GeoCoordinate coord = vertex.getCoordinate();
                minLat = Math.min(minLat, coord.getLatitude());
                maxLat = Math.max(maxLat, coord.getLatitude());
                minLon = Math.min(minLon, coord.getLongitude());
                maxLon = Math.max(maxLon, coord.getLongitude());
                minAlt = Math.min(minAlt, coord.getAltitude());
                maxAlt = Math.max(maxAlt, coord.getAltitude());
            }
            
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
        throw new UnsupportedOperationException("Operation not supported for a polygon.");
    }

    /**
         * Get the edges of the polygon
         */
        public List<SpatialLine> getEdges() {
            if (vertices.size() < 3) {
                throw new IllegalStateException("Polygon must have at least 3 vertices");
            }
            
            List<SpatialLine> edges = new ArrayList<>();
            for (int i = 0; i < vertices.size(); i++) {
                SpatialPoint start = vertices.get(i);
                SpatialPoint end = vertices.get((i + 1) % vertices.size());
                
                edges.add(SpatialLine.builder()
                    .id(id + "-edge-" + i)
                    .startPoint(start)
                    .endPoint(end)
                    .build());
            }
            
            return edges;
        }
        
        /**
         * Get a sample point inside the polygon (centroid)
         */
        public SpatialPoint getSamplePoint() {
            if (vertices.size() < 3) {
                throw new IllegalStateException("Polygon must have at least 3 vertices");
            }
            
            // Calculate centroid
            double sumLat = 0;
            double sumLon = 0;
            double sumAlt = 0;
            
            for (SpatialPoint vertex : vertices) {
                GeoCoordinate coord = vertex.getCoordinate();
                sumLat += coord.getLatitude();
                sumLon += coord.getLongitude();
                sumAlt += coord.getAltitude();
            }
            
            double avgLat = sumLat / vertices.size();
            double avgLon = sumLon / vertices.size();
            double avgAlt = sumAlt / vertices.size();
            
            return SpatialPoint.builder()
                .id(id + "-centroid")
                .coordinate(GeoCoordinate.builder()
                    .latitude(avgLat)
                    .longitude(avgLon)
                    .altitude(avgAlt)
                    .build())
                .build();
        }
        
        /**
         * Check if the polygon contains a point (using ray casting algorithm)
         */
        public boolean containsPoint(GeoCoordinate point) {
            if (vertices.size() < 3) {
                return false;
            }
            
            boolean inside = false;
            int n = vertices.size();
            
            for (int i = 0, j = n - 1; i < n; j = i++) {
                double lat_i = vertices.get(i).getCoordinate().getLatitude();
                double lon_i = vertices.get(i).getCoordinate().getLongitude();
                double lat_j = vertices.get(j).getCoordinate().getLatitude();
                double lon_j = vertices.get(j).getCoordinate().getLongitude();
                
                if (((lat_i > point.getLatitude()) != (lat_j > point.getLatitude())) &&
                    (point.getLongitude() < (lon_j - lon_i) * (point.getLatitude() - lat_i) / (lat_j - lat_i) + lon_i)) {
                    inside = !inside;
                }
            }
            
            return inside;
        }
    }