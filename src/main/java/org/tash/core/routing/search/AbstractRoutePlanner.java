package org.tash.core.routing.search;

import org.tash.core.routing.raw.RoutePlanner;
import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.spatial.SpatialPoint;
import org.tash.trajectory.TrajectorySegment;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.GreatCircleTrajectorySegment;
import org.tash.trajectory.TrajectoryType;

import java.time.ZonedDateTime;
import java.util.*;

/**
 * Abstract base class for route planners with shared functionality
 */
public abstract class AbstractRoutePlanner implements RoutePlanner {
    
    /**
     * Find a route between two points
     * 
     * @param start Starting point
     * @param end Ending point
     * @param constraints Set of constraints to consider
     * @return List of waypoints forming the route, or empty list if no route is found
     */
    @Override
    public abstract List<SpatialPoint> findRoute(SpatialPoint start, SpatialPoint end, Set<RoutingConstraint> constraints);
    
    /**
     * Find a route between two points with a maximum number of attempts
     * 
     * @param start Starting point
     * @param end Ending point
     * @param constraints Set of constraints to consider
     * @param maxAttempts Maximum number of attempts
     * @return List of waypoints forming the route, or empty list if no route is found
     */
    @Override
    public abstract List<SpatialPoint> findRoute(SpatialPoint start, SpatialPoint end, 
                                         Set<RoutingConstraint> constraints, int maxAttempts);
    
    /**
     * Convert a list of waypoints to trajectory segments with proper time calculations
     * 
     * @param waypoints List of waypoints forming the route
     * @return List of trajectory segments
     */
    @Override
    public List<TrajectorySegment> waypointsToTrajectory(List<SpatialPoint> waypoints) {
        List<TrajectorySegment> trajectorySegments = new ArrayList<>();
        
        if (waypoints.size() < 2) {
            return trajectorySegments; // Need at least 2 waypoints to form a segment
        }
        
        // Get the planning configuration
        PlanningConfig config = getConfig();
        double speedKnots = config.getSpeedKnots();
        
        // Start with current time for first segment
        ZonedDateTime currentTime = ZonedDateTime.now();
        
        // Create segments between consecutive waypoints
        for (int i = 0; i < waypoints.size() - 1; i++) {
            SpatialPoint from = waypoints.get(i);
            SpatialPoint to = waypoints.get(i + 1);
            
            // Calculate distance and time required for this segment
            double distanceNM = from.getCoordinate().distanceTo(to.getCoordinate());
            double durationHours = distanceNM / speedKnots;
            long durationSeconds = (long)(durationHours * 3600);
            
            // Calculate end time
            ZonedDateTime endTime = currentTime.plusSeconds(durationSeconds);
            
            // Determine altitude change to set appropriate trajectory type
            double altChange = Math.abs(to.getCoordinate().getAltitude() - from.getCoordinate().getAltitude());
            double climbRate = altChange / distanceNM; // feet per NM
            
            // Default to standard trajectory
            TrajectoryType trajectoryType = TrajectoryType.STANDARD;
            
            // If significant climb or descent is involved, mark accordingly
            if (climbRate > 300) { // More than 300 ft/NM is a significant climb/descent
                trajectoryType = TrajectoryType.CONTINGENCY;
            }
            
            // Create appropriate trajectory segment based on geometry
            TrajectorySegment segment;
            
            // Check if great circle path would be significantly different from straight line
            // For long-distance flights (over ~300 NM), use great circle path
            if (distanceNM > 300.0) {
                // Create a great circle trajectory segment
                segment = GreatCircleTrajectorySegment.builder()
                    .id("SEG-GC-" + i)
                    .source(from)
                    .target(to)
                    .startTime(currentTime)
                    .endTime(endTime)
                    .type(trajectoryType)
                    .numIntermediatePoints(10) // Use 10 intermediate points for path approximation
                    .build();
            } else {
                // For shorter distances, use linear trajectory segment
                segment = LinearTrajectorySegment.builder()
                    .id("SEG-" + i)
                    .source(from)
                    .target(to)
                    .startTime(currentTime)
                    .endTime(endTime)
                    .type(trajectoryType)
                    .build();
            }
            
            // Add segment to the list
            trajectorySegments.add(segment);
            
            // Update current time for next segment
            currentTime = endTime;
        }
        
        return trajectorySegments;
    }
    
    /**
     * Get the planning configuration
     * 
     * @return Planning configuration
     */
    public abstract PlanningConfig getConfig();
}