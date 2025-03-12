package org.tash.event.conflict;

import org.tash.data.Vector3D;
import org.tash.trajectory.TrajectorySegment;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

/**
     * Specialized conflict detection for emergency situations (relaxed standards)
     */
    public class EmergencyConflictDetection implements ConflictDetectionStrategy {
        // Reduced separation standards for emergency
        private static final double EMERGENCY_HORIZONTAL_SEPARATION_NM = 3.0;
        private static final double EMERGENCY_VERTICAL_SEPARATION_FT = 500.0;
        
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
            
            // Check if reduced separation standards are violated
            if (cpa.getHorizontalDistance() < EMERGENCY_HORIZONTAL_SEPARATION_NM) {
                if (Math.abs(cpa.getVerticalDistance()) < EMERGENCY_VERTICAL_SEPARATION_FT) {
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
        // Assuming TrajectorySegment has methods to get start and end positions as Vector3D
        Vector3D start1 = segment1.getStartPosition();
        Vector3D end1 = segment1.getEndPosition();
        Vector3D start2 = segment2.getStartPosition();
        Vector3D end2 = segment2.getEndPosition();

        // Calculate relative velocity
        Vector3D velocity1 = end1.subtract(start1);
        Vector3D velocity2 = end2.subtract(start2);
        Vector3D relativeVelocity = velocity1.subtract(velocity2);

        // Calculate relative position
        Vector3D relativePosition = start1.subtract(start2);

        // Calculate time of closest approach
        double t = -relativePosition.dotProduct(relativeVelocity) / relativeVelocity.magnitudeSquared();
        t = Math.max(0, Math.min(t, 1)); // Clamp t to the range [0, 1]

        // Calculate positions at closest approach
        Vector3D closestPoint1 = start1.add(velocity1.multiply(t));
        Vector3D closestPoint2 = start2.add(velocity2.multiply(t));

        // Calculate horizontal and vertical distances
        double horizontalDistance = Math.sqrt(Math.pow(closestPoint1.getX() - closestPoint2.getX(), 2) +
                Math.pow(closestPoint1.getY() - closestPoint2.getY(), 2));
        double verticalDistance = Math.abs(closestPoint1.getZ() - closestPoint2.getZ());

        // Calculate time of closest approach
        ZonedDateTime timeOfClosestApproach = segment1.getStartTime().plusSeconds((long) (t * segment1.getDuration().getSeconds()));

        // Create and return ClosestPointOfApproach object
        return ClosestPointOfApproach.builder()
                .time(timeOfClosestApproach)
                .horizontalDistance(horizontalDistance)
                .verticalDistance(verticalDistance)
                .build();
    }
}