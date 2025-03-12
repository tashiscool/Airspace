package org.tash.flight;

import lombok.*;
import org.tash.core.AirspaceElement;
import org.tash.core.ElementType;
import org.tash.data.BoundingBox;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.trajectory.TrajectorySegment;

import java.util.*;

/**
     * Represents a complete flight trajectory
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    @EqualsAndHashCode(onlyExplicitlyIncluded = true)
    public class FlightTrajectory implements AirspaceElement {
        @EqualsAndHashCode.Include
        private String id;
        private String callsign;
        
        @Builder.Default
        private List<TrajectorySegment> mainPath = new ArrayList<>();
        
        @Builder.Default
        private Map<SpatialPoint, List<List<TrajectorySegment>>> alternativePaths = new HashMap<>();
        
        @Builder.Default
        private Set<SpatialPoint> joinPoints = new HashSet<>();
        
        @Override
        public ElementType getElementType() {
            return ElementType.FLIGHT;
        }
        
        /**
         * Add a segment to the main path
         */
        public void addSegment(TrajectorySegment segment) {
            mainPath.add(segment);
        }
        
        /**
         * Get all segments, including those in alternative paths
         */
        public List<TrajectorySegment> getAllSegments() {
            List<TrajectorySegment> allSegments = new ArrayList<>(mainPath);
            
            for (List<List<TrajectorySegment>> forks : alternativePaths.values()) {
                for (List<TrajectorySegment> path : forks) {
                    allSegments.addAll(path);
                }
            }
            
            return allSegments;
        }
        
        /**
         * Add an alternative path from a fork point
         */
        public void addAlternativePath(SpatialPoint forkPoint, List<TrajectorySegment> path) {
            if (!alternativePaths.containsKey(forkPoint)) {
                alternativePaths.put(forkPoint, new ArrayList<>());
            }
            alternativePaths.get(forkPoint).add(path);
        }
        
        /**
         * Register a join point where alternative paths converge
         */
        public void joinAlternativePathsAt(SpatialPoint joinPoint) {
            joinPoints.add(joinPoint);
        }
        
        /**
         * Get all possible paths through the trajectory
         */
        public List<List<TrajectorySegment>> getAllPossiblePaths() {
            List<List<TrajectorySegment>> result = new ArrayList<>();
            
            // Start with just the main path
            result.add(new ArrayList<>(mainPath));
            
            // For each fork point, generate all combinations
            for (Map.Entry<SpatialPoint, List<List<TrajectorySegment>>> entry : alternativePaths.entrySet()) {
                SpatialPoint forkPoint = entry.getKey();
                List<List<TrajectorySegment>> forkOptions = entry.getValue();
                
                // Find fork point in main path
                int forkIndex = -1;
                for (int i = 0; i < mainPath.size(); i++) {
                    if (mainPath.get(i).getTarget().equals(forkPoint)) {
                        forkIndex = i;
                        break;
                    }
                }
                
                if (forkIndex >= 0) {
                    // Find join point
                    SpatialPoint joinPoint = null;
                    for (SpatialPoint jp : joinPoints) {
                        // Check if this join point is downstream from fork
                        for (int i = forkIndex + 1; i < mainPath.size(); i++) {
                            if (mainPath.get(i).getSource().equals(jp)) {
                                joinPoint = jp;
                                break;
                            }
                        }
                        if (joinPoint != null) break;
                    }
                    
                    // If we found a valid join point
                    if (joinPoint != null) {
                        int joinIndex = -1;
                        for (int i = forkIndex + 1; i < mainPath.size(); i++) {
                            if (mainPath.get(i).getSource().equals(joinPoint)) {
                                joinIndex = i;
                                break;
                            }
                        }
                        
                        // Create new path combinations
                        List<List<TrajectorySegment>> newCombinations = new ArrayList<>();
                        
                        for (List<TrajectorySegment> existingPath : result) {
                            // For each alternative
                            for (List<TrajectorySegment> alternative : forkOptions) {
                                List<TrajectorySegment> newPath = new ArrayList<>();
                                
                                // Add segments before fork
                                for (int i = 0; i <= forkIndex; i++) {
                                    newPath.add(existingPath.get(i));
                                }
                                
                                // Add alternative path
                                newPath.addAll(alternative);
                                
                                // Add segments after join
                                for (int i = joinIndex; i < existingPath.size(); i++) {
                                    newPath.add(existingPath.get(i));
                                }
                                
                                newCombinations.add(newPath);
                            }
                        }
                        
                        // Replace result with new combinations
                        if (!newCombinations.isEmpty()) {
                            result = newCombinations;
                        }
                    }
                }
            }
            
            return result;
        }
        
        /**
         * Get the bounding box for this flight trajectory
         */
        public BoundingBox getBoundingBox() {
            List<TrajectorySegment> segments = getAllSegments();
            if (segments.isEmpty()) {
                throw new IllegalStateException("Flight trajectory has no segments");
            }
            
            // Start with the first segment's bounding box
            BoundingBox bbox = segments.get(0).getBoundingBox();
            
            // Merge with all other segments
            for (int i = 1; i < segments.size(); i++) {
                bbox = bbox.merge(segments.get(i).getBoundingBox());
            }
            
            return bbox;
        }

    public boolean intersectsVolume(SpatialVolume volume) {
        for (TrajectorySegment segment : getAllSegments()) {
            if (segment.intersectsVolume(volume)) {
                return true;
            }
        }
        return false;
    }
}