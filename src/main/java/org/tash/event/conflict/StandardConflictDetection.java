package org.tash.event.conflict;

import org.tash.spatial.SpatialPoint;
import org.tash.trajectory.TrajectorySegment;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import static org.tash.AirspaceModel.MIN_HORIZONTAL_SEPARATION_NM;
import static org.tash.AirspaceModel.MIN_VERTICAL_SEPARATION_FT;

/**
     * Standard conflict detection implementation
     */
    public class StandardConflictDetection implements ConflictDetectionStrategy {
        @Override
        public List<SeparationConflict> detectConflicts(List<TrajectorySegment> segments) {
            List<SeparationConflict> conflicts = new ArrayList<>();
            
            // Check each pair of segments
            for (int i = 0; i < segments.size(); i++) {
                for (int j = i + 1; j < segments.size(); j++) {
                    SeparationConflict conflict = detectConflict(segments.get(i), segments.get(j));
                    if (conflict != null) {
                        conflicts.add(conflict);
                    }
                }
            }
            
            return conflicts;
        }
        
        @Override
        public SeparationConflict detectConflict(TrajectorySegment segment1, TrajectorySegment segment2) {
            // First check temporal overlap
            if (!segment1.timeOverlaps(segment2)) {
                return null;
            }
            
            // Calculate closest point of approach
            ClosestPointOfApproach cpa = calculateClosestPointOfApproach(segment1, segment2);
            
            // Check if separation standards are violated
            if (cpa.getHorizontalDistance() < MIN_HORIZONTAL_SEPARATION_NM) {
                if (Math.abs(cpa.getVerticalDistance()) < MIN_VERTICAL_SEPARATION_FT) {
                    // Create a conflict
                    return SeparationConflict.builder()
                        .segment1(segment1)
                        .segment2(segment2)
                        .time(cpa.getTime())
                        .horizontalSeparation(cpa.getHorizontalDistance())
                        .verticalSeparation(cpa.getVerticalDistance())
                        .build();
                }
            }
            
            return null;
        }

    private ClosestPointOfApproach calculateClosestPointOfApproach(TrajectorySegment segment1, TrajectorySegment segment2) {
        // Assuming TrajectorySegment has methods to get positions and times
        // and that positions are represented as some kind of vector (e.g., with x, y, z coordinates)

        // For simplicity, let's assume we have a method to calculate the distance between two points
        // and a method to interpolate positions at a given time

        // Find the time range overlap
        ZonedDateTime startTime = segment1.getStartTime().isAfter(segment2.getStartTime()) ? segment1.getStartTime() : segment2.getStartTime();
        ZonedDateTime endTime = segment1.getEndTime().isBefore(segment2.getEndTime()) ? segment1.getEndTime() : segment2.getEndTime();

        if (startTime.isAfter(endTime)) {
            // No overlap
            return null;
        }

        // Initialize the closest point of approach
        ClosestPointOfApproach closestPoint = null;
        double minDistance = Double.MAX_VALUE;

        // Check positions at discrete time intervals within the overlap period
        ZonedDateTime currentTime = startTime;
        while (!currentTime.isAfter(endTime)) {
            // Get positions at the current time
            SpatialPoint pos1 = segment1.getPointAtTime(currentTime);
            SpatialPoint pos2 = segment2.getPointAtTime(currentTime);

            // Calculate horizontal and vertical distances
            double horizontalDistance = calculateHorizontalDistance(pos1, pos2);
            double verticalDistance = calculateVerticalDistance(pos1, pos2);

            // Calculate the total distance
            double totalDistance = Math.sqrt(horizontalDistance * horizontalDistance + verticalDistance * verticalDistance);

            // Update the closest point of approach if this is the closest so far
            if (totalDistance < minDistance) {
                minDistance = totalDistance;
                closestPoint = ClosestPointOfApproach.builder()
                        .time(currentTime)
                        .horizontalDistance(horizontalDistance)
                        .verticalDistance(verticalDistance)
                        .build();
            }

            // Move to the next time step (e.g., 1 second later)
            currentTime = currentTime.plusSeconds(1);
        }

        return closestPoint;
    }

    // Placeholder methods for distance calculations
    private double calculateHorizontalDistance(SpatialPoint pos1, SpatialPoint pos2) {
            return Math.sqrt(Math.pow(pos1.getCoordinate().getLatitude() - pos2.getCoordinate().getLatitude(), 2) +
                    Math.pow(pos1.getCoordinate().getLongitude() - pos2.getCoordinate().getLongitude(), 2));
    }

    private double calculateVerticalDistance(SpatialPoint pos1, SpatialPoint pos2) {
        return Math.abs(pos1.getCoordinate().getAltitude() - pos2.getCoordinate().getAltitude());
    }
}