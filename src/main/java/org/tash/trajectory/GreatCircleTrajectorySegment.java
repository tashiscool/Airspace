package org.tash.trajectory;

import org.tash.data.GeoCoordinate;
import java.time.ZonedDateTime;
import java.util.Date;
import java.util.List;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.EqualsAndHashCode;
import lombok.NoArgsConstructor;
import lombok.experimental.SuperBuilder;
import org.tash.data.Vector3D;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.time.GreatCircleMotionModel;

import java.util.List;

/**
 * Represents a trajectory segment that follows a great circle path
 * More accurate than LinearTrajectorySegment for long distances
 */
@Data
@EqualsAndHashCode(callSuper = true)
@SuperBuilder
@NoArgsConstructor
@AllArgsConstructor
public class GreatCircleTrajectorySegment extends AbstractTrajectorySegment {
    private GreatCircleMotionModel motionModel;
    private int numIntermediatePoints; // Number of points to use for path approximation
    
    /**
     * Initialize motion model if not provided
     */
    public GreatCircleMotionModel getMotionModel() {
        if (motionModel == null) {
            motionModel = GreatCircleMotionModel.builder()
                .startPosition(source.getCoordinate())
                .endPosition(target.getCoordinate())
                .startTime(startTime)
                .endTime(endTime)
                .numIntermediatePoints(numIntermediatePoints > 0 ? numIntermediatePoints : 10)
                .build();
        }
        return motionModel;
    }
    
    @Override
    public SpatialPoint getPointAtFraction(double fraction) {
        // Clamp fraction between 0 and 1
        fraction = Math.max(0, Math.min(1, fraction));
        
        // Get path from motion model
        List<GeoCoordinate> path = getMotionModel().getPath();
        
        if (path.size() < 2) {
            // Fallback to direct interpolation if path not available
            GeoCoordinate interpolated = source.getCoordinate()
                .interpolate(target.getCoordinate(), fraction);
            
            return SpatialPoint.builder()
                .id(id + "-point-" + fraction)
                .coordinate(interpolated)
                .build();
        }
        
        // Find the appropriate segment in the path
        int index = (int) Math.floor(fraction * (path.size() - 1));
        index = Math.min(index, path.size() - 2); // Ensure we don't exceed array bounds
        
        double segmentFraction = (fraction * (path.size() - 1)) - index;
        
        // Interpolate between the two points in the segment
        GeoCoordinate pos1 = path.get(index);
        GeoCoordinate pos2 = path.get(index + 1);
        GeoCoordinate interpolated = pos1.interpolate(pos2, segmentFraction);
        
        return SpatialPoint.builder()
            .id(id + "-point-" + fraction)
            .coordinate(interpolated)
            .build();
    }

    @Override
    public SpatialPoint getPointAtTime(ZonedDateTime time) {
        return super.getPointAtTime(time);
    }

    @Override
    public SpatialLine toSpatialLine() {
        return super.toSpatialLine();
    }

    @Override
    public Vector3D getStartPosition() {
        return Vector3D.fromGeoCoordinate(source.getCoordinate());
    }

    @Override
    public Vector3D getEndPosition() {
        return Vector3D.fromGeoCoordinate(target.getCoordinate());
    }

    @Override
    public boolean intersectsVolume(SpatialVolume volume) {
        for (int i = 0; i < numIntermediatePoints; i++) {
            SpatialPoint point = getPointAtFraction(i / (double) numIntermediatePoints);
            if (volume.contains(point)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public Date getDuration() {
        return new Date(motionModel.getTimeInterval().getDuration().toMillis());
    }
}

