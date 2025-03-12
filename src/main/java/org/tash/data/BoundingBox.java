package org.tash.data;

import lombok.*;

import java.time.ZonedDateTime;

/**
     * Represents a bounding box for spatial queries
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class BoundingBox {
        private double minLat;
        private double maxLat;
        private double minLon;
        private double maxLon;
        private double minAlt;
        private double maxAlt;
        private ZonedDateTime startTime;
        private ZonedDateTime endTime;
        
        /**
         * Check if this bounding box intersects with another
         */
        public boolean intersects(BoundingBox other) {
            // Check spatial overlap
            boolean spatialOverlap = !(
                maxLat < other.minLat || minLat > other.maxLat ||
                maxLon < other.minLon || minLon > other.maxLon ||
                maxAlt < other.minAlt || minAlt > other.maxAlt
            );
            
            // Check temporal overlap if both have time data
            if (startTime != null && endTime != null && 
                other.startTime != null && other.endTime != null) {
                boolean temporalOverlap = !(
                    endTime.isBefore(other.startTime) || 
                    startTime.isAfter(other.endTime)
                );
                return spatialOverlap && temporalOverlap;
            }
            
            return spatialOverlap;
        }
        
        /**
         * Check if this bounding box contains a point
         */
        public boolean contains(GeoCoordinate point) {
            return point.getLatitude() >= minLat && point.getLatitude() <= maxLat &&
                   point.getLongitude() >= minLon && point.getLongitude() <= maxLon &&
                   point.getAltitude() >= minAlt && point.getAltitude() <= maxAlt;
        }
        
        /**
         * Merge this bounding box with another
         */
        public BoundingBox merge(BoundingBox other) {
            ZonedDateTime mergedStartTime = null;
            ZonedDateTime mergedEndTime = null;
            
            if (startTime != null && other.startTime != null) {
                mergedStartTime = startTime.isBefore(other.startTime) ? startTime : other.startTime;
                mergedEndTime = endTime.isAfter(other.endTime) ? endTime : other.endTime;
            } else if (startTime != null) {
                mergedStartTime = startTime;
                mergedEndTime = endTime;
            } else if (other.startTime != null) {
                mergedStartTime = other.startTime;
                mergedEndTime = other.endTime;
            }
            
            return BoundingBox.builder()
                .minLat(Math.min(minLat, other.minLat))
                .maxLat(Math.max(maxLat, other.maxLat))
                .minLon(Math.min(minLon, other.minLon))
                .maxLon(Math.max(maxLon, other.maxLon))
                .minAlt(Math.min(minAlt, other.minAlt))
                .maxAlt(Math.max(maxAlt, other.maxAlt))
                .startTime(mergedStartTime)
                .endTime(mergedEndTime)
                .build();
        }

    public boolean contains(double lat, double lon, double alt) {
        return lat >= minLat && lat <= maxLat &&
                lon >= minLon && lon <= maxLon &&
                alt >= minAlt && alt <= maxAlt;
    }

    public boolean contains(BoundingBox boundingBox) {
        return contains(boundingBox.minLat, boundingBox.minLon, boundingBox.minAlt) &&
                contains(boundingBox.maxLat, boundingBox.maxLon, boundingBox.maxAlt);
    }
}