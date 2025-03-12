package org.tash.core.routing.search.astar;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.core.routing.raw.RoutingNode;
import org.tash.core.routing.search.AbstractRoutePlanner;
import org.tash.core.routing.search.PlanningConfig;
import org.tash.spatial.SpatialPoint;
import org.tash.trajectory.TrajectorySegment;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectoryType;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.*;

/**
 * A* pathfinding algorithm with custom heuristics for airspace navigation
 */
public class AStarRoutePlanner extends AbstractRoutePlanner {
    private final NodeGenerator nodeGenerator;
    private final HeuristicFunction heuristicFunction;
    private final AStarConfig config;
    
    /**
     * Create an A* planner with default configuration
     */
    public AStarRoutePlanner() {
        this(new DefaultNodeGenerator(), new GeodeticHeuristic(), new AStarConfig());
    }
    
    /**
     * Create an A* planner with custom settings
     */
    public AStarRoutePlanner(NodeGenerator nodeGenerator, HeuristicFunction heuristicFunction, AStarConfig config) {
        this.nodeGenerator = nodeGenerator;
        this.heuristicFunction = heuristicFunction;
        this.config = config;
    }
    
    @Override
    public List<SpatialPoint> findRoute(SpatialPoint start, SpatialPoint end, Set<RoutingConstraint> constraints) {
        return findRoute(start, end, constraints, config.getMaxIterations());
    }
    
    @Override
    public List<SpatialPoint> findRoute(SpatialPoint start, SpatialPoint end,
                                        Set<RoutingConstraint> constraints, int maxAttempts) {
        // Open set: nodes to be evaluated
        PriorityQueue<RoutingNode> openSet = new PriorityQueue<>();
        
        // Closed set: nodes already evaluated
        Set<String> closedSet = new HashSet<>();
        
        // Start with the initial node
        double initialHeuristic = heuristicFunction.estimate(start, end, constraints);
        RoutingNode startNode = new RoutingNode(start, null, 0, initialHeuristic);
        openSet.add(startNode);
        
        int iterations = 0;
        
        while (!openSet.isEmpty() && iterations < maxAttempts) {
            iterations++;
            
            // Get node with lowest f-cost
            RoutingNode current = openSet.poll();
            
            // Add to closed set
            closedSet.add(getNodeKey(current.getPoint()));
            
            // Check if we've reached the goal
            if (isGoalReached(current.getPoint(), end)) {
                return reconstructPath(current);
            }
            
            // Generate neighbor nodes
            List<RoutingNode> neighbors = nodeGenerator.generateNeighbors(current, end, constraints, 
                                                                          heuristicFunction, config);
            
            // Process each neighbor
            for (RoutingNode neighbor : neighbors) {
                String neighborKey = getNodeKey(neighbor.getPoint());
                
                // Skip if already evaluated
                if (closedSet.contains(neighborKey)) {
                    continue;
                }
                
                // Check if this node is already in the open set with a better path
                boolean shouldAdd = true;
                for (RoutingNode existingNode : openSet) {
                    if (existingNode.getPoint().equals(neighbor.getPoint()) && 
                        existingNode.getGCost() <= neighbor.getGCost()) {
                        shouldAdd = false;
                        break;
                    }
                }
                
                if (shouldAdd) {
                    openSet.add(neighbor);
                }
            }
        }
        
        // No path found
        return Collections.emptyList();
    }
    
    @Override
    public List<TrajectorySegment> waypointsToTrajectory(List<SpatialPoint> waypoints) {
        List<TrajectorySegment> segments = new ArrayList<>();
        
        if (waypoints.size() < 2) {
            return segments;
        }
        
        // Current time for creating segments
        ZonedDateTime currentTime = ZonedDateTime.now();
        
        // Create segments between consecutive waypoints
        for (int i = 0; i < waypoints.size() - 1; i++) {
            SpatialPoint start = waypoints.get(i);
            SpatialPoint end = waypoints.get(i + 1);
            
            // Calculate segment duration based on distance
            double distanceNM = start.getCoordinate().distanceTo(end.getCoordinate());
            double durationMinutes = distanceNM / config.getSpeedKnots() * 60.0;
            
            // Create a linear trajectory segment
            LinearTrajectorySegment segment = LinearTrajectorySegment.builder()
                .id("ROUTE-SEG-" + i)
                .source(start)
                .target(end)
                .startTime(currentTime)
                .endTime(currentTime.plusSeconds((long)(durationMinutes * 60)))
                .type(TrajectoryType.STANDARD)
                .build();
            
            segments.add(segment);
            
            // Update current time for next segment
            currentTime = segment.getEndTime();
        }
        
        return segments;
    }

    @Override
    public PlanningConfig getConfig() {
        return config;
    }

    /**
     * Check if the current point is close enough to the goal
     */
    private boolean isGoalReached(SpatialPoint current, SpatialPoint goal) {
        double distance = current.getCoordinate().distanceTo(goal.getCoordinate());
        return distance <= config.getGoalReachedThresholdNM();
    }
    
    /**
     * Reconstruct the path from the goal node back to the start
     */
    private List<SpatialPoint> reconstructPath(RoutingNode goalNode) {
        List<SpatialPoint> path = new ArrayList<>();
        RoutingNode current = goalNode;
        
        // Traverse back to the start
        while (current != null) {
            path.add(current.getPoint());
            current = current.getParent();
        }
        
        // Reverse to get path from start to goal
        Collections.reverse(path);
        
        // Path smoothing if enabled
        if (config.isPathSmoothingEnabled()) {
            path = smoothPath(path);
        }
        
        return path;
    }
    
    /**
     * Smooth the path by removing unnecessary waypoints
     */
    private List<SpatialPoint> smoothPath(List<SpatialPoint> path) {
        if (path.size() <= 2) {
            return path;
        }
        
        List<SpatialPoint> smoothedPath = new ArrayList<>();
        smoothedPath.add(path.get(0));
        
        int currentIndex = 0;
        
        while (currentIndex < path.size() - 1) {
            int farthestVisible = currentIndex + 1;
            
            // Find the farthest visible waypoint
            for (int i = currentIndex + 2; i < path.size(); i++) {
                SpatialPoint start = path.get(currentIndex);
                SpatialPoint end = path.get(i);
                
                // Check if direct path is collision-free
                boolean isVisible = true;
                for (int j = currentIndex + 1; j < i; j++) {
                    SpatialPoint intermediate = path.get(j);
                    
                    // Check if intermediate point is reasonably close to the direct path
                    double crossTrack = intermediate.getCoordinate()
                        .crossTrackDistanceToPath(start.getCoordinate(), end.getCoordinate());
                    
                    if (Math.abs(crossTrack) > config.getPathSmoothingToleranceNM()) {
                        isVisible = false;
                        break;
                    }
                }
                
                if (isVisible) {
                    farthestVisible = i;
                }
            }
            
            // Add the farthest visible waypoint
            if (farthestVisible != currentIndex + 1) {
                smoothedPath.add(path.get(farthestVisible));
            } else {
                smoothedPath.add(path.get(farthestVisible));
            }
            
            currentIndex = farthestVisible;
        }
        
        return smoothedPath;
    }
    
    /**
     * Generate a unique key for a node based on its position
     */
    private String getNodeKey(SpatialPoint point) {
        // Use a grid-based system for better spatial hashing
        GeoCoordinate coord = point.getCoordinate();
        double gridSizeDeg = config.getGridSizeNM() / 60.0;  // approximate conversion
        
        int latGrid = (int)(coord.getLatitude() / gridSizeDeg);
        int lonGrid = (int)(coord.getLongitude() / gridSizeDeg);
        int altGrid = (int)(coord.getAltitude() / config.getAltitudeGridFt());
        
        return latGrid + ":" + lonGrid + ":" + altGrid;
    }
    

}