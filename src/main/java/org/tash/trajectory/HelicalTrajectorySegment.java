package org.tash.trajectory;

import org.tash.data.GeoCoordinate;
import org.tash.data.Vector3D;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.time.HelicalMotionModel;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.EqualsAndHashCode;
import lombok.NoArgsConstructor;
import lombok.experimental.SuperBuilder;

import java.util.Date;
import java.util.List;

/**
 * Represents a helical trajectory segment (climbing or descending turn)
 * Combines circular motion in horizontal plane with linear motion in vertical plane
 */
@Data
@EqualsAndHashCode(callSuper = true)
@SuperBuilder
@NoArgsConstructor
@AllArgsConstructor
public class HelicalTrajectorySegment extends AbstractTrajectorySegment {
    private SpatialPoint center;       // Center of the helix (horizontal)
    private double radius;             // Radius of the helix in nautical miles
    private double startAngle;         // Start angle in degrees
    private double endAngle;           // End angle in degrees
    private double startAltitude;      // Start altitude in feet
    private double endAltitude;        // End altitude in feet
    private boolean clockwise;         // Direction of the helix
    private HelicalMotionModel motionModel;
    
    /**
     * Initialize motion model if not provided
     */
    public HelicalMotionModel getMotionModel() {
        if (motionModel == null) {
            motionModel = HelicalMotionModel.builder()
                .center(center.getCoordinate())
                .radius(radius)
                .startAngle(startAngle)
                .endAngle(endAngle)
                .startAltitude(startAltitude)
                .endAltitude(endAltitude)
                .clockwise(clockwise)
                .startTime(startTime)
                .endTime(endTime)
                .build();
        }
        return motionModel;
    }
    
    @Override
    public SpatialPoint getPointAtFraction(double fraction) {
        // Clamp fraction between 0 and 1
        fraction = Math.max(0, Math.min(1, fraction));
        
        // Get position at fraction
        GeoCoordinate position = getMotionModel().getPositionAtFraction(fraction);
        
        return SpatialPoint.builder()
            .id(id + "-point-" + fraction)
            .coordinate(position)
            .build();
    }

    @Override
    public Vector3D getStartPosition() {
        GeoCoordinate startCoordinate = getMotionModel().getPositionAtFraction(0);
        return new Vector3D(startCoordinate.getLatitude(), startCoordinate.getLongitude(), startCoordinate.getAltitude());
    }

    @Override
    public Vector3D getEndPosition() {
        GeoCoordinate endCoordinate = getMotionModel().getPositionAtFraction(1);
        return new Vector3D(endCoordinate.getLatitude(), endCoordinate.getLongitude(), endCoordinate.getAltitude());
    }

    @Override
    public boolean intersectsVolume(SpatialVolume volume) {
        List<GeoCoordinate> helixPoints = getMotionModel().getHelixPoints(100);
        for (GeoCoordinate point : helixPoints) {
            if (volume.contains(point)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public Date getDuration() {
        return java.util.Date.from(java.time.Instant.ofEpochMilli(java.time.Duration.between(startTime, endTime).toMillis()));
    }

    /**
     * Get the number of complete turns in this helical segment
     * @return Number of turns (can be fractional)
     */
    public double getNumberOfTurns() {
        double angleDiff = Math.abs(normalizeAngleDifference(endAngle - startAngle));
        return angleDiff / 360.0;
    }
    
    /**
     * Get the vertical rate of the helix in feet per minute
     * @return Vertical rate (positive for climb, negative for descent)
     */
    public double getVerticalRate() {
        double durationMinutes = java.time.Duration.between(startTime, endTime).toMinutes();
        if (durationMinutes <= 0) return 0;
        
        return (endAltitude - startAltitude) / durationMinutes;
    }
    
    /**
     * Get the ground track distance (horizontal distance traveled)
     * @return Distance in nautical miles
     */
    public double getGroundTrackDistance() {
        double angleDiff = Math.abs(normalizeAngleDifference(endAngle - startAngle));
        return 2 * Math.PI * radius * (angleDiff / 360.0);
    }
    
    /**
     * Calculate the 3D distance traveled along the helix
     * @return Distance in nautical miles
     */
    public double get3DDistance() {
        double horizontalDist = getGroundTrackDistance();
        // Convert vertical distance from feet to nautical miles (1 NM = 6076.12 feet)
        double verticalDist = Math.abs(endAltitude - startAltitude) / 6076.12;
        
        // Pythagorean theorem for 3D distance
        return Math.sqrt(horizontalDist * horizontalDist + verticalDist * verticalDist);
    }
    
    /**
     * Normalize angle difference to handle the -180 to +180 range properly
     */
    private double normalizeAngleDifference(double angleDiff) {
        angleDiff = angleDiff % 360;
        if (angleDiff > 180) angleDiff -= 360;
        if (angleDiff < -180) angleDiff += 360;
        return angleDiff;
    }
}

