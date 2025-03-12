package org.tash.core.routing.raw;

import org.tash.spatial.SpatialPoint;
import org.tash.trajectory.TrajectorySegment;

import java.util.List;
import java.util.Set;

/**
 * Interface for route planning algorithms
 */
public interface RoutePlanner {
    /**
     * Find a route between two points
     * 
     * @param start Starting point
     * @param end Ending point
     * @param constraints Set of constraints to consider
     * @return List of waypoints forming the route, or empty list if no route is found
     */
    List<SpatialPoint> findRoute(SpatialPoint start, SpatialPoint end, Set<RoutingConstraint> constraints);
    
    /**
     * Find a route between two points with a maximum number of attempts
     * 
     * @param start Starting point
     * @param end Ending point
     * @param constraints Set of constraints to consider
     * @param maxAttempts Maximum number of attempts
     * @return List of waypoints forming the route, or empty list if no route is found
     */
    List<SpatialPoint> findRoute(SpatialPoint start, SpatialPoint end, 
                                Set<RoutingConstraint> constraints, int maxAttempts);
    
    /**
     * Convert a list of waypoints to trajectory segments
     * 
     * @param waypoints List of waypoints
     * @return List of trajectory segments
     */
    List<TrajectorySegment> waypointsToTrajectory(List<SpatialPoint> waypoints);
}

