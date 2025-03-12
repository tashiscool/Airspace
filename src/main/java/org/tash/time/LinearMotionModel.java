package org.tash.time;

import lombok.*;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

@Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class LinearMotionModel implements MotionModel {
        private GeoCoordinate startPosition;
        private GeoCoordinate endPosition;
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
            double fraction = totalMillis > 0 ? elapsedMillis / totalMillis : 0;
            
            // Linear interpolation between start and end
            return startPosition.interpolate(endPosition, fraction);
        }
        
        @Override
        public TimeInterval getTimeInterval() {
            return new TimeInterval(startTime, endTime);
        }
    }