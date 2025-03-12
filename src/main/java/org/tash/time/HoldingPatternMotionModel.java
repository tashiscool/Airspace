package org.tash.time;

import lombok.*;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialPoint;

import java.time.ZonedDateTime;

/**
     * Motion model for holding pattern
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class HoldingPatternMotionModel implements MotionModel {
        private GeoCoordinate centerPoint;
        private double legLength;  // nautical miles
        private double heading;    // degrees
        private int turns;
        private ZonedDateTime startTime;
        private ZonedDateTime endTime;
        private GeoCoordinate entryPoint;
        private GeoCoordinate exitPoint;
        
        @Override
        public GeoCoordinate getPositionAt(ZonedDateTime time) {
            // Check if time is outside the segment timeframe
            if (time.isBefore(startTime)) {
                return entryPoint;
            } else if (time.isAfter(endTime)) {
                return exitPoint;
            }
            
            // Calculate position in holding pattern based on time
            double totalDuration = java.time.Duration.between(startTime, endTime).toMillis();
            double elapsedTime = java.time.Duration.between(startTime, time).toMillis();
            double normalizedTime = (elapsedTime / totalDuration) * turns;
            
            // Determine which part of the holding pattern we're in
            double patternPosition = normalizedTime % 1.0;
            
            // Calculate position along the holding pattern
            // A holding pattern consists of 4 segments: outbound leg, right turn, inbound leg, right turn
            double heading_rad = Math.toRadians(heading);
            double lat = centerPoint.getLatitude();
            double lon = centerPoint.getLongitude();
            double alt = centerPoint.getAltitude();
            
            if (patternPosition < 0.25) {
                // Outbound leg
                double legFraction = patternPosition * 4;
                double offset = legLength * legFraction;
                
                // Calculate position along outbound leg
                double lat_offset = Math.sin(heading_rad) * offset / 60.0;  // 1 minute of latitude = 1 NM
                double lon_offset = Math.cos(heading_rad) * offset / (60.0 * Math.cos(Math.toRadians(lat)));
                
                lat = centerPoint.getLatitude() + lat_offset;
                lon = centerPoint.getLongitude() + lon_offset;
            } else if (patternPosition < 0.5) {
                // First 180° turn
                double turnFraction = (patternPosition - 0.25) * 4;
                double turnAngle = Math.PI * turnFraction;
                double radius = legLength / 2;
                
                // Calculate position in the turn
                double turnHeading = heading_rad + Math.PI / 2; // 90° from heading
                double x = radius * Math.cos(turnHeading + turnAngle);
                double y = radius * Math.sin(turnHeading + turnAngle);
                
                // Convert to lat/lon
                lat = centerPoint.getLatitude() + y / 60.0;
                lon = centerPoint.getLongitude() + x / (60.0 * Math.cos(Math.toRadians(lat)));
            } else if (patternPosition < 0.75) {
                // Inbound leg
                double legFraction = (patternPosition - 0.5) * 4;
                double offset = legLength * (1 - legFraction);
                
                // Calculate position along inbound leg
                double lat_offset = Math.sin(heading_rad) * offset / 60.0;
                double lon_offset = Math.cos(heading_rad) * offset / (60.0 * Math.cos(Math.toRadians(lat)));
                
                lat = centerPoint.getLatitude() + lat_offset;
                lon = centerPoint.getLongitude() + lon_offset;
            } else {
                // Second 180° turn
                double turnFraction = (patternPosition - 0.75) * 4;
                double turnAngle = Math.PI * turnFraction;
                double radius = legLength / 2;
                
                // Calculate position in the turn
                double turnHeading = heading_rad - Math.PI / 2; // -90° from heading
                double x = radius * Math.cos(turnHeading + turnAngle);
                double y = radius * Math.sin(turnHeading + turnAngle);
                
                // Convert to lat/lon
                lat = centerPoint.getLatitude() + y / 60.0;
                lon = centerPoint.getLongitude() + x / (60.0 * Math.cos(Math.toRadians(lat)));
            }
            
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

    public SpatialPoint getPositionAt(double fraction) {
            GeoCoordinate geoCoordinate = getPositionAt(startTime.plusNanos((long) ((endTime.toEpochSecond() - startTime.toEpochSecond()) * fraction * 1e9)));
            return SpatialPoint.builder()
                    .id("holding-pattern-" + fraction)
                    .coordinate(geoCoordinate)
                    .build();
    }
}
    