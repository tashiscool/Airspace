package org.tash.time;

import lombok.*;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

/**
     * Motion model for curved trajectory (using Bezier curve)
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class CurvedMotionModel implements MotionModel {
        private GeoCoordinate startPosition;
        private GeoCoordinate endPosition;
        private GeoCoordinate controlPoint;
        private ZonedDateTime startTime;
        private ZonedDateTime endTime;
        
        @Override
        public GeoCoordinate getPositionAt(ZonedDateTime time) {
            // Check if time is outside the segment timeframe
            if (time.isBefore(startTime)) {
                return startPosition;
            } else if (time.isAfter(endTime)) {
                return endPosition;
            }
            
            // Calculate the time fraction (0 to 1)
            double totalMillis = java.time.Duration.between(startTime, endTime).toMillis();
            double elapsedMillis = java.time.Duration.between(startTime, time).toMillis();
            double t = totalMillis > 0 ? elapsedMillis / totalMillis : 0;
            
            // Quadratic Bezier curve formula: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
            double mt = 1 - t;
            double mt2 = mt * mt;
            double t2 = t * t;
            
            double lat = mt2 * startPosition.getLatitude() + 
                        2 * mt * t * controlPoint.getLatitude() + 
                        t2 * endPosition.getLatitude();
                        
            double lon = mt2 * startPosition.getLongitude() + 
                        2 * mt * t * controlPoint.getLongitude() + 
                        t2 * endPosition.getLongitude();
                        
            double alt = mt2 * startPosition.getAltitude() + 
                        2 * mt * t * controlPoint.getAltitude() + 
                        t2 * endPosition.getAltitude();
            
            return GeoCoordinate.builder()
                .latitude(lat)
                .longitude(lon)
                .altitude(alt)
                .build();
        }
        
        @Override
        public TimeInterval getTimeInterval() {
            return new TimeInterval(startTime, endTime);
        }
    }