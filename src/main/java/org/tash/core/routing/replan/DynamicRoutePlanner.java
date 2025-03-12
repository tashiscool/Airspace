package org.tash.core.routing.replan;

import org.tash.core.routing.raw.AbstractRoutingConstraint;
import org.tash.core.routing.raw.RestrictedAirspaceConstraint;
import org.tash.core.routing.raw.RoutePlanner;
import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.core.routing.search.astar.AStarRoutePlanner;
import org.tash.core.routing.search.rrt.RRTRoutePlanner;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;
import org.tash.trajectory.GreatCircleTrajectorySegment;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectorySegment;
import org.tash.data.GeoCoordinate;
import org.tash.trajectory.TrajectoryType;

import java.util.*;
import java.util.concurrent.CopyOnWriteArrayList;
import java.time.ZonedDateTime;

/**
 * Dynamic route replanning capabilities to adapt routes to evolving constraints
 */
public class DynamicRoutePlanner {
    private final RoutePlanner basePlanner;
    private final ReplanningSensorConfig sensorConfig;
    private final List<RouteChangeListener> changeListeners;
    private final List<DynamicConstraint> dynamicConstraints;
    
    /**
     * Create a dynamic route planner with default A* planner
     */
    public DynamicRoutePlanner() {
        this(new AStarRoutePlanner(), new ReplanningSensorConfig());
    }
    
    /**
     * Create a dynamic route planner with custom planner and sensor configuration
     */
    public DynamicRoutePlanner(RoutePlanner basePlanner, ReplanningSensorConfig sensorConfig) {
        this.basePlanner = basePlanner;
        this.sensorConfig = sensorConfig;
        this.changeListeners = new CopyOnWriteArrayList<>();
        this.dynamicConstraints = new CopyOnWriteArrayList<>();
    }
    
    /**
     * Add a constraint that can change dynamically
     * 
     * @param constraint The dynamic constraint to add
     */
    public void addDynamicConstraint(DynamicConstraint constraint) {
        dynamicConstraints.add(constraint);
    }
    
    /**
     * Remove a dynamic constraint
     * 
     * @param constraint The constraint to remove
     * @return True if the constraint was removed
     */
    public boolean removeDynamicConstraint(DynamicConstraint constraint) {
        return dynamicConstraints.remove(constraint);
    }
    
    /**
     * Add a listener for route change events
     * 
     * @param listener The listener to add
     */
    public void addRouteChangeListener(RouteChangeListener listener) {
        changeListeners.add(listener);
    }
    
    /**
     * Remove a route change listener
     * 
     * @param listener The listener to remove
     * @return True if the listener was removed
     */
    public boolean removeRouteChangeListener(RouteChangeListener listener) {
        return changeListeners.remove(listener);
    }
    
    /**
     * Plan an initial route with current constraints
     * 
     * @param start Starting point
     * @param end Ending point
     * @param staticConstraints Set of static constraints
     * @return The planned route or empty list if no route is found
     */
    public DynamicRoute planInitialRoute(SpatialPoint start, SpatialPoint end, 
                                        Set<RoutingConstraint> staticConstraints) {
        // Combine static and dynamic constraints
        Set<RoutingConstraint> allConstraints = new HashSet<>(staticConstraints);
        for (DynamicConstraint constraint : dynamicConstraints) {
            allConstraints.add(constraint.getCurrentConstraint());
        }
        
        // Plan the route
        List<SpatialPoint> waypoints = basePlanner.findRoute(start, end, allConstraints);
        
        if (waypoints.isEmpty()) {
            return new DynamicRoute(Collections.emptyList(), start, end, false);
        }
        
        // Create trajectory segments
        List<TrajectorySegment> segments = basePlanner.waypointsToTrajectory(waypoints);
        
        // Create and return the dynamic route
        return new DynamicRoute(segments, start, end, true);
    }
    
    /**
     * Check if the current route needs replanning due to constraint changes
     * 
     * @param route The current route
     * @param currentPosition Current position
     * @param staticConstraints Set of static constraints
     * @return True if the route needs replanning
     */
    public boolean routeNeedsReplanning(DynamicRoute route, SpatialPoint currentPosition, 
                                       Set<RoutingConstraint> staticConstraints) {
        // If route is empty or already invalid, it needs replanning
        if (!route.isValid() || route.getSegments().isEmpty()) {
            return true;
        }
        
        // First, check if we're still on the route
        boolean onRoute = isPositionOnRoute(currentPosition, route);
        if (!onRoute && sensorConfig.isRerouteWhenOffTrack()) {
            return true;
        }
        
        // Find remaining segments (from current position forward)
        List<TrajectorySegment> remainingSegments = getRemainingSegments(route, currentPosition);
        
        // Combine static and dynamic constraints
        Set<RoutingConstraint> allConstraints = new HashSet<>(staticConstraints);
        for (DynamicConstraint constraint : dynamicConstraints) {
            allConstraints.add(constraint.getCurrentConstraint());
        }
        
        // Check each segment against all constraints
        for (TrajectorySegment segment : remainingSegments) {
            for (RoutingConstraint constraint : allConstraints) {
                if (constraint.isViolated(segment.getSource(), segment.getTarget())) {
                    // If this is a hard constraint, we need immediate replanning
                    if (constraint.isHardConstraint()) {
                        return true;
                    }
                    
                    // For soft constraints, check if penalty exceeds threshold
                    double penalty = constraint.getPenalty();
                    if (penalty > sensorConfig.getConstraintPenaltyThreshold()) {
                        return true;
                    }
                }
            }
        }
        
        return false;
    }
    
    /**
     * Replan a route from the current position
     * 
     * @param route The current route
     * @param currentPosition Current position
     * @param staticConstraints Set of static constraints
     * @return The replanned route
     */
    public DynamicRoute replanRoute(DynamicRoute route, SpatialPoint currentPosition, 
                                   Set<RoutingConstraint> staticConstraints) {
        // Create a notification of replanning start
        RouteReplanningEvent startEvent = new RouteReplanningEvent(
            RouteReplanningEvent.Type.REPLANNING_STARTED,
            route,
            currentPosition,
            "Route replanning initiated",
            ZonedDateTime.now()
        );
        
        // Notify listeners
        notifyListeners(startEvent);
        
        // Combine static and dynamic constraints
        Set<RoutingConstraint> allConstraints = new HashSet<>(staticConstraints);
        for (DynamicConstraint constraint : dynamicConstraints) {
            allConstraints.add(constraint.getCurrentConstraint());
        }
        
        // Plan a new route from current position to destination
        List<SpatialPoint> newWaypoints = basePlanner.findRoute(
            currentPosition, 
            route.getDestination(), 
            allConstraints
        );
        
        // Create result based on success or failure
        DynamicRoute result;
        String message;
        
        if (newWaypoints.isEmpty()) {
            // No valid route found
            result = new DynamicRoute(Collections.emptyList(), currentPosition, route.getDestination(), false);
            message = "Unable to find valid route to destination";
        } else {
            // Create trajectory segments
            List<TrajectorySegment> segments = basePlanner.waypointsToTrajectory(newWaypoints);
            result = new DynamicRoute(segments, currentPosition, route.getDestination(), true);
            message = "Route successfully replanned";
        }
        
        // Create a notification of replanning completion
        RouteReplanningEvent completeEvent = new RouteReplanningEvent(
            result.isValid() ? RouteReplanningEvent.Type.REPLANNING_SUCCEEDED : RouteReplanningEvent.Type.REPLANNING_FAILED,
            result,
            currentPosition,
            message,
            ZonedDateTime.now()
        );
        
        // Notify listeners
        notifyListeners(completeEvent);
        
        return result;
    }
    
    /**
     * Check if a position is on the current route
     * 
     * @param position The position to check
     * @param route The route to check against
     * @return True if the position is on the route within tolerance
     */
    private boolean isPositionOnRoute(SpatialPoint position, DynamicRoute route) {
        GeoCoordinate posCoord = position.getCoordinate();
        
        // Check each segment
        for (TrajectorySegment segment : route.getSegments()) {
            SpatialPoint from = segment.getSource();
            SpatialPoint to = segment.getTarget();
            
            // Check if position is on this segment
            boolean onSegment = posCoord.isOnPathSegment(
                from.getCoordinate(), 
                to.getCoordinate(), 
                sensorConfig.getOnRouteToleranceNM()
            );
            
            if (onSegment) {
                return true;
            }
        }
        
        return false;
    }
    
    /**
     * Get the remaining segments of a route from the current position
     * 
     * @param route The route
     * @param currentPosition Current position
     * @return List of remaining segments
     */
    private List<TrajectorySegment> getRemainingSegments(DynamicRoute route, SpatialPoint currentPosition) {
        List<TrajectorySegment> segments = route.getSegments();
        if (segments.isEmpty()) {
            return Collections.emptyList();
        }
        
        // Find the segment closest to the current position
        int closestIndex = 0;
        double minDistance = Double.MAX_VALUE;
        GeoCoordinate posCoord = currentPosition.getCoordinate();
        
        for (int i = 0; i < segments.size(); i++) {
            TrajectorySegment segment = segments.get(i);
            SpatialPoint from = segment.getSource();
            SpatialPoint to = segment.getTarget();
            
            // Find the closest point on this segment
            GeoCoordinate closestPoint = posCoord.closestPointOnPath(
                from.getCoordinate(), 
                to.getCoordinate()
            );
            
            double distance = posCoord.distanceTo(closestPoint);
            if (distance < minDistance) {
                minDistance = distance;
                closestIndex = i;
            }
        }
        
        // Return segments from the closest one to the end
        return segments.subList(closestIndex, segments.size());
    }
    
    /**
     * Notify all listeners of a route change event
     * 
     * @param event The event to notify about
     */
    private void notifyListeners(RouteReplanningEvent event) {
        for (RouteChangeListener listener : changeListeners) {
            listener.onRouteChange(event);
        }
    }

    /**
     * Convert a list of waypoints to trajectory segments with proper time calculations
     *
     * @param waypoints List of waypoints forming the route
     * @return List of trajectory segments
     */
    public List<TrajectorySegment> waypointsToTrajectory(List<SpatialPoint> waypoints) {
        List<TrajectorySegment> trajectorySegments = new ArrayList<>();

        if (waypoints.size() < 2) {
            return trajectorySegments; // Need at least 2 waypoints to form a segment
        }

        // Configuration values
        double speedKnots = 500.0; // Default speed in knots
        if (basePlanner instanceof AStarRoutePlanner) {
            speedKnots = ((AStarRoutePlanner) basePlanner).getConfig().getSpeedKnots();
        } else if (basePlanner instanceof RRTRoutePlanner) {
            speedKnots = ((RRTRoutePlanner) basePlanner).getConfig().getSpeedKnots();
        }

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

            // Get trajectory type
            TrajectoryType trajectoryType = TrajectoryType.STANDARD;

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

    public DynamicRoute dynamicReplan(List<SpatialPoint> routeInitial, SpatialPoint offTrackPoint, Set<RoutingConstraint> staticConstraints)
    {
        // Combine static and dynamic constraints
        Set<RoutingConstraint> allConstraints = new HashSet<>(staticConstraints);
        for (DynamicConstraint constraint : dynamicConstraints) {
            allConstraints.add(constraint.getCurrentConstraint());
        }

        // Plan a new route from current position to destination
        List<SpatialPoint> newWaypoints = basePlanner.findRoute(
                offTrackPoint,
                routeInitial.get(routeInitial.size() - 1),
                allConstraints
        );

        // Create result based on success or failure
        DynamicRoute result;
        String message;

        if (newWaypoints.isEmpty()) {
            // No valid route found
            result = new DynamicRoute(Collections.emptyList(), offTrackPoint, routeInitial.get(routeInitial.size() - 1), false);
            message = "Unable to find valid route to destination";
        } else {
            // Create trajectory segments
            List<TrajectorySegment> segments = basePlanner.waypointsToTrajectory(newWaypoints);
            result = new DynamicRoute(segments, offTrackPoint, routeInitial.get(routeInitial.size() - 1), true);
            message = "Route successfully replanned";
        }

        return result;
    }

    /**
     * Interface for dynamic constraints that can change over time
     */
    public interface DynamicConstraint {
        /**
         * Get the current state of the constraint
         * 
         * @return The current constraint
         */
        RoutingConstraint getCurrentConstraint();
        
        /**
         * Get the ID of this dynamic constraint
         * 
         * @return Constraint ID
         */
        String getId();
        
        /**
         * Predict the future state of the constraint
         * 
         * @param timeInFuture Time in the future
         * @return The predicted constraint
         */
        RoutingConstraint predictConstraintAt(ZonedDateTime timeInFuture);
    }
    
    /**
     * Weather volume that moves and changes as a dynamic constraint
     */
    public static class MovingWeatherConstraint implements DynamicConstraint {
        private final String id;
        private final SpatialVolume initialVolume;
        private final double speedKnots;
        private final double bearingDegrees;
        private final ZonedDateTime creationTime;
        private final double penalty;
        private final boolean isHardConstraint;
        
        public MovingWeatherConstraint(String id, SpatialVolume initialVolume, double speedKnots, 
                                     double bearingDegrees, double penalty, boolean isHardConstraint) {
            this.id = id;
            this.initialVolume = initialVolume;
            this.speedKnots = speedKnots;
            this.bearingDegrees = bearingDegrees;
            this.creationTime = ZonedDateTime.now();
            this.penalty = penalty;
            this.isHardConstraint = isHardConstraint;
        }
        
        @Override
        public String getId() {
            return id;
        }
        
        @Override
        public RoutingConstraint getCurrentConstraint() {
            // Calculate time since creation
            ZonedDateTime now = ZonedDateTime.now();
            long secondsElapsed = java.time.Duration.between(creationTime, now).getSeconds();
            
            // Calculate distance moved
            double hoursElapsed = secondsElapsed / 3600.0;
            double distanceNM = speedKnots * hoursElapsed;
            
            // Calculate new position
            SpatialVolume currentVolume = moveVolume(initialVolume, distanceNM, bearingDegrees);
            
            // Create and return constraint
            return new RestrictedAirspaceConstraint(currentVolume, penalty, isHardConstraint);
        }
        
        @Override
        public RoutingConstraint predictConstraintAt(ZonedDateTime futureTime) {
            // Calculate time difference
            long secondsElapsed = java.time.Duration.between(creationTime, futureTime).getSeconds();
            
            // Calculate distance moved
            double hoursElapsed = secondsElapsed / 3600.0;
            double distanceNM = speedKnots * hoursElapsed;
            
            // Calculate new position
            SpatialVolume predictedVolume = moveVolume(initialVolume, distanceNM, bearingDegrees);
            
            // Create and return constraint
            return new RestrictedAirspaceConstraint(predictedVolume, penalty, isHardConstraint);
        }
        
        /**
         * Move a spatial volume by a distance and bearing
         */
        private SpatialVolume moveVolume(SpatialVolume volume, double distanceNM, double bearingDegrees) {
            // Get the base polygon
            SpatialPolygon basePolygon = volume.getBasePolygon();
            
            // Move each vertex
            List<SpatialPoint> newVertices = new ArrayList<>();
            for (SpatialPoint vertex : basePolygon.getVertices()) {
                GeoCoordinate oldCoord = vertex.getCoordinate();
                GeoCoordinate newCoord = oldCoord.destinationPoint(distanceNM, bearingDegrees);
                
                SpatialPoint newVertex = SpatialPoint.builder()
                    .id(vertex.getId() + "-moved")
                    .coordinate(newCoord)
                    .build();
                
                newVertices.add(newVertex);
            }
            
            // Create new polygon
            SpatialPolygon newPolygon = SpatialPolygon.builder()
                .id(basePolygon.getId() + "-moved")
                .vertices(newVertices)
                .build();
            
            // Create new volume
            return SpatialVolume.builder()
                .id(volume.getId() + "-moved")
                .basePolygon(newPolygon)
                .lowerAltitude(volume.getLowerAltitude())
                .upperAltitude(volume.getUpperAltitude())
                .startTime(volume.getStartTime())
                .endTime(volume.getEndTime())
                .build();
        }
    }
    
    /**
     * Dynamic time window constraint that activates/deactivates based on time
     */
    public static class TimeWindowConstraint implements DynamicConstraint {
        private final String id;
        private final RoutingConstraint baseConstraint;
        private final ZonedDateTime activeStartTime;
        private final ZonedDateTime activeEndTime;
        
        public TimeWindowConstraint(String id, RoutingConstraint baseConstraint, 
                                  ZonedDateTime activeStartTime, ZonedDateTime activeEndTime) {
            this.id = id;
            this.baseConstraint = baseConstraint;
            this.activeStartTime = activeStartTime;
            this.activeEndTime = activeEndTime;
        }
        
        @Override
        public String getId() {
            return id;
        }
        
        @Override
        public RoutingConstraint getCurrentConstraint() {
            ZonedDateTime now = ZonedDateTime.now();
            
            // Check if constraint is active
            boolean isActive = !now.isBefore(activeStartTime) && !now.isAfter(activeEndTime);
            
            if (isActive) {
                return baseConstraint;
            } else {
                // Return a null constraint (never violated)
                return new AbstractRoutingConstraint(0, false) {
                    @Override
                    public boolean isViolated(SpatialPoint from, SpatialPoint to) {
                        return false;
                    }
                };
            }
        }
        
        @Override
        public RoutingConstraint predictConstraintAt(ZonedDateTime futureTime) {
            // Check if constraint will be active
            boolean willBeActive = !futureTime.isBefore(activeStartTime) && !futureTime.isAfter(activeEndTime);
            
            if (willBeActive) {
                return baseConstraint;
            } else {
                // Return a null constraint (never violated)
                return new AbstractRoutingConstraint(0, false) {
                    @Override
                    public boolean isViolated(SpatialPoint from, SpatialPoint to) {
                        return false;
                    }
                };
            }
        }
    }
    
    /**
     * Expanding or contracting airspace constraint
     */
    public static class ChangingSizeConstraint implements DynamicConstraint {
        private final String id;
        private final SpatialVolume initialVolume;
        private final double expansionRatePercent; // Percent per hour
        private final ZonedDateTime creationTime;
        private final double penalty;
        private final boolean isHardConstraint;
        private final double maxExpansionPercent;
        
        public ChangingSizeConstraint(String id, SpatialVolume initialVolume, 
                                    double expansionRatePercent, double maxExpansionPercent,
                                    double penalty, boolean isHardConstraint) {
            this.id = id;
            this.initialVolume = initialVolume;
            this.expansionRatePercent = expansionRatePercent;
            this.maxExpansionPercent = maxExpansionPercent;
            this.creationTime = ZonedDateTime.now();
            this.penalty = penalty;
            this.isHardConstraint = isHardConstraint;
        }
        
        @Override
        public String getId() {
            return id;
        }
        
        @Override
        public RoutingConstraint getCurrentConstraint() {
            // Calculate time since creation
            ZonedDateTime now = ZonedDateTime.now();
            long secondsElapsed = java.time.Duration.between(creationTime, now).getSeconds();
            
            // Calculate expansion factor
            double hoursElapsed = secondsElapsed / 3600.0;
            double expansionPercent = expansionRatePercent * hoursElapsed;
            
            // Limit to max expansion
            expansionPercent = Math.min(expansionPercent, maxExpansionPercent);
            
            // Calculate new size
            double scaleFactor = 1.0 + (expansionPercent / 100.0);
            
            if (expansionRatePercent < 0) {
                // For contraction, ensure we don't go below 0
                scaleFactor = Math.max(0.01, scaleFactor);
            }
            
            // Create scaled volume
            SpatialVolume currentVolume = scaleVolume(initialVolume, scaleFactor);
            
            // Create and return constraint
            return new RestrictedAirspaceConstraint(currentVolume, penalty, isHardConstraint);
        }
        
        @Override
        public RoutingConstraint predictConstraintAt(ZonedDateTime futureTime) {
            // Calculate time difference
            long secondsElapsed = java.time.Duration.between(creationTime, futureTime).getSeconds();
            
            // Calculate expansion factor
            double hoursElapsed = secondsElapsed / 3600.0;
            double expansionPercent = expansionRatePercent * hoursElapsed;
            
            // Limit to max expansion
            expansionPercent = Math.min(expansionPercent, maxExpansionPercent);
            
            // Calculate new size
            double scaleFactor = 1.0 + (expansionPercent / 100.0);
            
            if (expansionRatePercent < 0) {
                // For contraction, ensure we don't go below 0
                scaleFactor = Math.max(0.01, scaleFactor);
            }
            
            // Create scaled volume
            SpatialVolume predictedVolume = scaleVolume(initialVolume, scaleFactor);
            
            // Create and return constraint
            return new RestrictedAirspaceConstraint(predictedVolume, penalty, isHardConstraint);
        }
        
        /**
         * Scale a spatial volume by a factor
         */
        private SpatialVolume scaleVolume(SpatialVolume volume, double scaleFactor) {
            // Get the base polygon
            SpatialPolygon basePolygon = volume.getBasePolygon();
            
            // Calculate centroid
            SpatialPoint centroid = basePolygon.getSamplePoint(); // This returns the centroid
            GeoCoordinate center = centroid.getCoordinate();
            
            // Scale each vertex relative to centroid
            List<SpatialPoint> newVertices = new ArrayList<>();
            for (SpatialPoint vertex : basePolygon.getVertices()) {
                GeoCoordinate oldCoord = vertex.getCoordinate();
                
                // Calculate distance and bearing from center
                double distance = center.distanceTo(oldCoord);
                double bearing = center.initialBearingTo(oldCoord);
                
                // Scale the distance
                double newDistance = distance * scaleFactor;
                
                // Calculate new position
                GeoCoordinate newCoord = center.destinationPoint(newDistance, bearing);
                
                SpatialPoint newVertex = SpatialPoint.builder()
                    .id(vertex.getId() + "-scaled")
                    .coordinate(newCoord)
                    .build();
                
                newVertices.add(newVertex);
            }
            
            // Create new polygon
            SpatialPolygon newPolygon = SpatialPolygon.builder()
                .id(basePolygon.getId() + "-scaled")
                .vertices(newVertices)
                .build();
            
            // Create new volume with scaled altitude ranges
            double altMidpoint = (volume.getLowerAltitude() + volume.getUpperAltitude()) / 2;
            double altRange = volume.getUpperAltitude() - volume.getLowerAltitude();
            double scaledAltRange = altRange * scaleFactor;
            
            return SpatialVolume.builder()
                .id(volume.getId() + "-scaled")
                .basePolygon(newPolygon)
                .lowerAltitude(altMidpoint - scaledAltRange / 2)
                .upperAltitude(altMidpoint + scaledAltRange / 2)
                .startTime(volume.getStartTime())
                .endTime(volume.getEndTime())
                .build();
        }
    }
    
    /**
     * Represents a dynamic route with replanning capabilities
     */
    public static class DynamicRoute {
        private final List<TrajectorySegment> segments;
        private final SpatialPoint origin;
        private final SpatialPoint destination;
        private final boolean isValid;
        
        public DynamicRoute(List<TrajectorySegment> segments, SpatialPoint origin, 
                          SpatialPoint destination, boolean isValid) {
            this.segments = segments;
            this.origin = origin;
            this.destination = destination;
            this.isValid = isValid;
        }
        
        public List<TrajectorySegment> getSegments() {
            return segments;
        }
        
        public SpatialPoint getOrigin() {
            return origin;
        }
        
        public SpatialPoint getDestination() {
            return destination;
        }
        
        public boolean isValid() {
            return isValid;
        }
        
        /**
         * Get all waypoints of the route
         */
        public List<SpatialPoint> getWaypoints() {
            if (segments.isEmpty()) {
                return Collections.emptyList();
            }
            
            List<SpatialPoint> waypoints = new ArrayList<>();
            
            // Add the first waypoint
            waypoints.add(segments.get(0).getSource());
            
            // Add all target waypoints
            for (TrajectorySegment segment : segments) {
                waypoints.add(segment.getTarget());
            }
            
            return waypoints;
        }
        
        /**
         * Calculate the total distance of the route
         * 
         * @return Total distance in nautical miles
         */
        public double getTotalDistance() {
            double totalDistance = 0;
            
            for (TrajectorySegment segment : segments) {
                SpatialPoint from = segment.getSource();
                SpatialPoint to = segment.getTarget();
                totalDistance += from.getCoordinate().distanceTo(to.getCoordinate());
            }
            
            return totalDistance;
        }
        
        /**
         * Calculate the estimated time of arrival
         * 
         * @param speedKnots Speed in knots
         * @return Estimated time of arrival
         */
        public ZonedDateTime getEstimatedTimeOfArrival(double speedKnots) {
            if (segments.isEmpty()) {
                return ZonedDateTime.now();
            }
            
            // Get departure and arrival times from segments
            ZonedDateTime departureTime = segments.get(0).getStartTime();
            ZonedDateTime arrivalTime = segments.get(segments.size() - 1).getEndTime();
            
            return arrivalTime;
        }
    }
    
    /**
     * Event for route replanning
     */
    public static class RouteReplanningEvent {
        public enum Type {
            REPLANNING_STARTED,
            REPLANNING_SUCCEEDED,
            REPLANNING_FAILED,
            CONSTRAINT_CHANGED
        }
        
        private final Type type;
        private final DynamicRoute route;
        private final SpatialPoint currentPosition;
        private final String message;
        private final ZonedDateTime timestamp;
        
        public RouteReplanningEvent(Type type, DynamicRoute route, SpatialPoint currentPosition, 
                                  String message, ZonedDateTime timestamp) {
            this.type = type;
            this.route = route;
            this.currentPosition = currentPosition;
            this.message = message;
            this.timestamp = timestamp;
        }
        
        public Type getType() {
            return type;
        }
        
        public DynamicRoute getRoute() {
            return route;
        }
        
        public SpatialPoint getCurrentPosition() {
            return currentPosition;
        }
        
        public String getMessage() {
            return message;
        }
        
        public ZonedDateTime getTimestamp() {
            return timestamp;
        }
    }
    
    /**
     * Interface for listening to route change events
     */
    public interface RouteChangeListener {
        /**
         * Called when a route change occurs
         * 
         * @param event The route change event
         */
        void onRouteChange(RouteReplanningEvent event);
    }
    
    /**
     * Configuration for replanning sensors
     */
    public static class ReplanningSensorConfig {
        private double onRouteToleranceNM = 2.0;
        private boolean rerouteWhenOffTrack = true;
        private double constraintPenaltyThreshold = 10.0;
        private double lookaheadTimeMinutes = 15.0;
        private boolean predictiveReplanning = true;
        
        // Getters and setters
        public double getOnRouteToleranceNM() {
            return onRouteToleranceNM;
        }
        
        public void setOnRouteToleranceNM(double onRouteToleranceNM) {
            this.onRouteToleranceNM = onRouteToleranceNM;
        }
        
        public boolean isRerouteWhenOffTrack() {
            return rerouteWhenOffTrack;
        }
        
        public void setRerouteWhenOffTrack(boolean rerouteWhenOffTrack) {
            this.rerouteWhenOffTrack = rerouteWhenOffTrack;
        }
        
        public double getConstraintPenaltyThreshold() {
            return constraintPenaltyThreshold;
        }
        
        public void setConstraintPenaltyThreshold(double constraintPenaltyThreshold) {
            this.constraintPenaltyThreshold = constraintPenaltyThreshold;
        }
        
        public double getLookaheadTimeMinutes() {
            return lookaheadTimeMinutes;
        }
        
        public void setLookaheadTimeMinutes(double lookaheadTimeMinutes) {
            this.lookaheadTimeMinutes = lookaheadTimeMinutes;
        }
        
        public boolean isPredictiveReplanning() {
            return predictiveReplanning;
        }
        
        public void setPredictiveReplanning(boolean predictiveReplanning) {
            this.predictiveReplanning = predictiveReplanning;
        }
        
        /**
         * Create a builder for fluent configuration
         */
        public static Builder builder() {
            return new Builder();
        }
        
        /**
         * Builder for ReplanningSensorConfig
         */
        public static class Builder {
            private final ReplanningSensorConfig config = new ReplanningSensorConfig();
            
            public Builder onRouteToleranceNM(double onRouteToleranceNM) {
                config.setOnRouteToleranceNM(onRouteToleranceNM);
                return this;
            }
            
            public Builder rerouteWhenOffTrack(boolean rerouteWhenOffTrack) {
                config.setRerouteWhenOffTrack(rerouteWhenOffTrack);
                return this;
            }
            
            public Builder constraintPenaltyThreshold(double constraintPenaltyThreshold) {
                config.setConstraintPenaltyThreshold(constraintPenaltyThreshold);
                return this;
            }
            
            public Builder lookaheadTimeMinutes(double lookaheadTimeMinutes) {
                config.setLookaheadTimeMinutes(lookaheadTimeMinutes);
                return this;
            }
            
            public Builder predictiveReplanning(boolean predictiveReplanning) {
                config.setPredictiveReplanning(predictiveReplanning);
                return this;
            }
            
            public ReplanningSensorConfig build() {
                return config;
            }
        }
    }
    
    /**
     * Manager for predictive route planning
     */
    public static class PredictiveRoutePlanner {
        private final DynamicRoutePlanner basePlanner;
        private final long predictionIntervalMillis;
        private final int numPredictionSteps;
        private final double speedKnots;
        
        public PredictiveRoutePlanner(DynamicRoutePlanner basePlanner, long predictionIntervalMillis, 
                                     int numPredictionSteps, double speedKnots) {
            this.basePlanner = basePlanner;
            this.predictionIntervalMillis = predictionIntervalMillis;
            this.numPredictionSteps = numPredictionSteps;
            this.speedKnots = speedKnots;
        }
        
        /**
         * Predict if the route will need replanning in the future
         * 
         * @param route Current route
         * @param currentPosition Current position
         * @param staticConstraints Static constraints
         * @return Predicted time when replanning will be needed, or null if no replanning is predicted
         */
        public ZonedDateTime predictReplanningTime(DynamicRoute route, SpatialPoint currentPosition, 
                                                Set<RoutingConstraint> staticConstraints) {
            // Get dynamic constraints
            List<DynamicConstraint> dynamicConstraints = basePlanner.dynamicConstraints;
            
            // If no dynamic constraints, no need for predictive replanning
            if (dynamicConstraints.isEmpty()) {
                return null;
            }
            
            // Check future time steps
            ZonedDateTime now = ZonedDateTime.now();
            
            for (int i = 1; i <= numPredictionSteps; i++) {
                // Calculate future time
                ZonedDateTime futureTime = now.plusNanos(predictionIntervalMillis * i * 1_000_000);
                
                // Predict future position
                SpatialPoint predictedPosition = predictPositionAt(route, currentPosition, futureTime);
                if (predictedPosition == null) {
                    // Reached end of route
                    break;
                }
                
                // Collect predicted constraints
                Set<RoutingConstraint> predictedConstraints = new HashSet<>(staticConstraints);
                for (DynamicConstraint constraint : dynamicConstraints) {
                    predictedConstraints.add(constraint.predictConstraintAt(futureTime));
                }
                
                // Create temporary route from predicted position to destination
                List<TrajectorySegment> remainingSegments = getRemainingSegmentsAfter(route, predictedPosition);
                if (remainingSegments.isEmpty()) {
                    // No segments left
                    break;
                }
                
                DynamicRoute predictedRoute = new DynamicRoute(
                    remainingSegments, 
                    predictedPosition, 
                    route.getDestination(),
                    true
                );
                
                // Check if replanning would be needed
                boolean needsReplanning = checkConstraintViolations(predictedRoute, predictedConstraints);
                if (needsReplanning) {
                    return futureTime;
                }
            }
            
            // No replanning predicted
            return null;
        }
        
        /**
         * Predict position along the route at a future time
         */
        private SpatialPoint predictPositionAt(DynamicRoute route, SpatialPoint currentPosition, 
                                            ZonedDateTime futureTime) {
            // Get remaining segments
            List<TrajectorySegment> segments = getRemainingSegmentsFrom(route, currentPosition);
            if (segments.isEmpty()) {
                return null;
            }
            
            // Calculate time difference
            ZonedDateTime now = ZonedDateTime.now();
            double minutesDiff = java.time.Duration.between(now, futureTime).toMinutes();
            
            // Calculate distance that will be traveled
            double distanceNM = (speedKnots / 60.0) * minutesDiff;
            
            // Track accumulated distance
            double accumulatedDistance = 0;
            
            // Find the segment and position
            for (TrajectorySegment segment : segments) {
                SpatialPoint from = segment.getSource();
                SpatialPoint to = segment.getTarget();
                
                double segmentDistance = from.getCoordinate().distanceTo(to.getCoordinate());
                
                if (accumulatedDistance + segmentDistance >= distanceNM) {
                    // This is the segment where we'll be
                    double remainingDistance = distanceNM - accumulatedDistance;
                    double fraction = remainingDistance / segmentDistance;
                    
                    // Interpolate position
                    double bearing = from.getCoordinate().initialBearingTo(to.getCoordinate());
                    GeoCoordinate predicted = from.getCoordinate().destinationPoint(remainingDistance, bearing);
                    
                    return SpatialPoint.builder()
                        .id("predicted-" + UUID.randomUUID().toString())
                        .coordinate(predicted)
                        .build();
                }
                
                accumulatedDistance += segmentDistance;
            }
            
            // If we've gone beyond the route, return the destination
            return route.getDestination();
        }
        
        /**
         * Get segments starting from a position
         */
        private List<TrajectorySegment> getRemainingSegmentsFrom(DynamicRoute route, SpatialPoint position) {
            List<TrajectorySegment> segments = route.getSegments();
            if (segments.isEmpty()) {
                return Collections.emptyList();
            }
            
            // Find the segment closest to the position
            int closestIndex = 0;
            double minDistance = Double.MAX_VALUE;
            GeoCoordinate posCoord = position.getCoordinate();
            
            for (int i = 0; i < segments.size(); i++) {
                TrajectorySegment segment = segments.get(i);
                SpatialPoint from = segment.getSource();
                SpatialPoint to = segment.getTarget();
                
                // Find the closest point on this segment
                GeoCoordinate closestPoint = posCoord.closestPointOnPath(
                    from.getCoordinate(), 
                    to.getCoordinate()
                );
                
                double distance = posCoord.distanceTo(closestPoint);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestIndex = i;
                }
            }
            
            // Return segments from the closest one to the end
            return segments.subList(closestIndex, segments.size());
        }
        
        /**
         * Get segments after a position
         */
        private List<TrajectorySegment> getRemainingSegmentsAfter(DynamicRoute route, SpatialPoint position) {
            List<TrajectorySegment> segments = route.getSegments();
            if (segments.isEmpty()) {
                return Collections.emptyList();
            }
            
            // Find the segment closest to the position
            int closestIndex = 0;
            double minDistance = Double.MAX_VALUE;
            GeoCoordinate posCoord = position.getCoordinate();
            
            for (int i = 0; i < segments.size(); i++) {
                TrajectorySegment segment = segments.get(i);
                SpatialPoint from = segment.getSource();
                SpatialPoint to = segment.getTarget();
                
                // Find the closest point on this segment
                GeoCoordinate closestPoint = posCoord.closestPointOnPath(
                    from.getCoordinate(), 
                    to.getCoordinate()
                );
                
                double distance = posCoord.distanceTo(closestPoint);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestIndex = i;
                }
            }
            
            // Return segments after the closest one (not including it)
            if (closestIndex < segments.size() - 1) {
                return segments.subList(closestIndex + 1, segments.size());
            } else {
                return Collections.emptyList();
            }
        }
        
        /**
         * Check if a route violates any constraints
         */
        private boolean checkConstraintViolations(DynamicRoute route, Set<RoutingConstraint> constraints) {
            for (TrajectorySegment segment : route.getSegments()) {
                for (RoutingConstraint constraint : constraints) {
                    if (constraint.isViolated(segment.getSource(), segment.getTarget())) {
                        if (constraint.isHardConstraint()) {
                            return true;
                        }
                    }
                }
            }
            
            return false;
        }
    }
}