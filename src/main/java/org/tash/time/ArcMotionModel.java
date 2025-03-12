package org.tash.time;

import lombok.*;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

/**
 * Motion model for arc-based trajectory
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ArcMotionModel implements MotionModel {
    private GeoCoordinate center;
    private double radius;
    private double startAngle;
    private double endAngle;
    private boolean isClockwise;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;
    
    @Override
    public GeoCoordinate getPositionAt(ZonedDateTime time) {
        // Check if time is outside the segment timeframe
        if (time.isBefore(startTime)) {
            // Calculate the start position
            return calculatePositionAtAngle(startAngle);
        } else if (time.isAfter(endTime)) {
            // Calculate the end position
            return calculatePositionAtAngle(endAngle);
        }
        
        // Normalize angles to 0-360 range
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
        
        // Calculate the time fraction (0 to 1)
        double totalMillis = java.time.Duration.between(startTime, endTime).toMillis();
        double elapsedMillis = java.time.Duration.between(startTime, time).toMillis();
        double fraction = totalMillis > 0 ? elapsedMillis / totalMillis : 0;
        
        // Calculate angle at fraction
        double angleOffset = isClockwise ? -fraction * sweep : fraction * sweep;
        double angle = normalizeAngle(normStart + angleOffset);
        
        // Calculate position at that angle
        return calculatePositionAtAngle(angle);
    }
    
    /**
     * Calculate position at a specific angle
     */
    private GeoCoordinate calculatePositionAtAngle(double angle) {
        double angleRad = Math.toRadians(angle);
        
        double lat = center.getLatitude();
        double lon = center.getLongitude();
        
        // Convert radius from nautical miles to appropriate deltas
        double latDelta = radius * Math.sin(angleRad) / 60.0;
        double lonDelta = radius * Math.cos(angleRad) / (60.0 * Math.cos(Math.toRadians(lat)));
        
        return GeoCoordinate.builder()
            .latitude(lat + latDelta)
            .longitude(lon + lonDelta)
            .altitude(center.getAltitude())
            .build();
    }
    
    /**
     * Normalize angle to 0-360 range
     */
    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }
    
    @Override
    public TimeInterval getTimeInterval() {
        return new TimeInterval(startTime, endTime);
    }
}