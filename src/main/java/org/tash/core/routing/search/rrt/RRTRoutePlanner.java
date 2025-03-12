package org.tash.core.routing.search.rrt;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.core.routing.search.AbstractRoutePlanner;
import org.tash.core.routing.search.PlanningConfig;
import org.tash.spatial.SpatialPoint;
import org.tash.trajectory.TrajectorySegment;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectoryType;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.*;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Rapidly-exploring Random Trees (RRT) algorithm for complex airspace navigation
 * This implementation includes RRT* optimizations for path quality
 */
public class RRTRoutePlanner extends AbstractRoutePlanner {
    private final RRTConfig config;
    
    /**
     * Create an RRT planner with default configuration
     */
    public RRTRoutePlanner() {
        this(new RRTConfig());
    }
    
    /**
     * Create an RRT planner with custom configuration
     */
    public RRTRoutePlanner(RRTConfig config) {
        this.config = config;
    }
    
    @Override
    public List<SpatialPoint> findRoute(SpatialPoint start, SpatialPoint end, Set<RoutingConstraint> constraints) {
        return findRoute(start, end, constraints, config.getMaxIterations());
    }
    
    @Override
    public List<SpatialPoint> findRoute(SpatialPoint start, SpatialPoint end,
                                        Set<RoutingConstraint> constraints, int maxAttempts) {
        // Initialize the tree with the start node
        RRTNode root = new RRTNode(start, null);
        List<RRTNode> tree = new ArrayList<>();
        tree.add(root);
        
        // Goal bias parameters
        double goalBias = config.getGoalBias();
        
        // Keep track of the node closest to the goal
        RRTNode closestToGoal = root;
        double closestDistance = root.getPoint().getCoordinate().distanceTo(end.getCoordinate());
        
        // Track if we've reached the goal
        boolean goalReached = false;
        RRTNode goalNode = null;
        
        // Main RRT algorithm loop
        for (int i = 0; i < maxAttempts; i++) {
            // With some probability, try to extend towards the goal
            SpatialPoint randomPoint;
            if (ThreadLocalRandom.current().nextDouble() < goalBias) {
                randomPoint = end;
            } else {
                // Generate a random point in the search space
                randomPoint = sampleRandomPoint(start, end, constraints);
            }
            
            // Find the nearest node in the tree
            RRTNode nearest = findNearestNode(tree, randomPoint);
            
            // Extend the tree towards the random point
            RRTNode newNode = extend(nearest, randomPoint, constraints);
            
            if (newNode != null) {
                // If using RRT*, perform optimization steps
                if (config.isUseRRTStar()) {
                    optimizeTree(tree, newNode, constraints);
                }
                
                // Add the new node to the tree
                tree.add(newNode);
                
                // Check if this node is closer to the goal
                double distanceToGoal = newNode.getPoint().getCoordinate().distanceTo(end.getCoordinate());
                if (distanceToGoal < closestDistance) {
                    closestToGoal = newNode;
                    closestDistance = distanceToGoal;
                }
                
                // Check if we've reached the goal
                if (distanceToGoal <= config.getGoalReachedThresholdNM()) {
                    goalReached = true;
                    goalNode = newNode;
                    
                    // If we're not using RRT*, we can stop now
                    if (!config.isUseRRTStar()) {
                        break;
                    }
                }
            }
        }
        
        // If goal was reached, extract the path
        if (goalReached) {
            return extractPath(goalNode);
        } else {
            // If goal wasn't reached, return the best path found
            return extractPath(closestToGoal);
        }
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
                .id("RRT-SEG-" + i)
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
     * Generate a random point in the search space
     */
    private SpatialPoint sampleRandomPoint(SpatialPoint start, SpatialPoint end, Set<RoutingConstraint> constraints) {
        // Create random sampling boundaries based on start and end points
        GeoCoordinate startCoord = start.getCoordinate();
        GeoCoordinate endCoord = end.getCoordinate();
        
        // Get min/max values with some padding
        double minLat = Math.min(startCoord.getLatitude(), endCoord.getLatitude()) - config.getSearchSpacePaddingDeg();
        double maxLat = Math.max(startCoord.getLatitude(), endCoord.getLatitude()) + config.getSearchSpacePaddingDeg();
        double minLon = Math.min(startCoord.getLongitude(), endCoord.getLongitude()) - config.getSearchSpacePaddingDeg();
        double maxLon = Math.max(startCoord.getLongitude(), endCoord.getLongitude()) + config.getSearchSpacePaddingDeg();
        double minAlt = Math.min(startCoord.getAltitude(), endCoord.getAltitude()) - config.getSearchSpaceAltPaddingFt();
        double maxAlt = Math.max(startCoord.getAltitude(), endCoord.getAltitude()) + config.getSearchSpaceAltPaddingFt();
        
        // Generate random coordinates
        double randomLat = minLat + ThreadLocalRandom.current().nextDouble() * (maxLat - minLat);
        double randomLon = minLon + ThreadLocalRandom.current().nextDouble() * (maxLon - minLon);
        double randomAlt = minAlt + ThreadLocalRandom.current().nextDouble() * (maxAlt - minAlt);
        
        // Create and return the random point
        return SpatialPoint.builder()
            .id("random-" + UUID.randomUUID().toString())
            .coordinate(GeoCoordinate.builder()
                .latitude(randomLat)
                .longitude(randomLon)
                .altitude(randomAlt)
                .build())
            .build();
    }
    
    /**
     * Find the nearest node in the tree to a given point
     */
    private RRTNode findNearestNode(List<RRTNode> tree, SpatialPoint point) {
        RRTNode nearest = null;
        double minDistance = Double.MAX_VALUE;
        
        for (RRTNode node : tree) {
            double distance = node.getPoint().getCoordinate().distanceTo(point.getCoordinate());
            if (distance < minDistance) {
                minDistance = distance;
                nearest = node;
            }
        }
        
        return nearest;
    }
    
    /**
     * Extend the tree from nearest node towards the random point
     */
    private RRTNode extend(RRTNode nearest, SpatialPoint target, Set<RoutingConstraint> constraints) {
        // Calculate direction and distance
        GeoCoordinate fromCoord = nearest.getPoint().getCoordinate();
        GeoCoordinate toCoord = target.getCoordinate();
        
        double distance = fromCoord.distanceTo(toCoord);
        double bearing = fromCoord.initialBearingTo(toCoord);
        
        // If the distance is greater than the step size, limit it
        double actualDistance = Math.min(distance, config.getStepSizeNM());
        
        // Create a new point at the step distance
        GeoCoordinate newCoord;
        if (actualDistance < distance) {
            // Create an intermediate point
            newCoord = fromCoord.destinationPoint(actualDistance, bearing);
            
            // Interpolate altitude
            double altFraction = actualDistance / distance;
            double newAlt = fromCoord.getAltitude() + 
                           (toCoord.getAltitude() - fromCoord.getAltitude()) * altFraction;
            
            newCoord = GeoCoordinate.builder()
                .latitude(newCoord.getLatitude())
                .longitude(newCoord.getLongitude())
                .altitude(newAlt)
                .build();
        } else {
            // Use the target point directly
            newCoord = toCoord;
        }
        
        // Create the new point
        SpatialPoint newPoint = SpatialPoint.builder()
            .id("rrt-" + UUID.randomUUID().toString())
            .coordinate(newCoord)
            .build();
        
        // Check for constraint violations
        for (RoutingConstraint constraint : constraints) {
            if (constraint.isHardConstraint() && constraint.isViolated(nearest.getPoint(), newPoint)) {
                // Hard constraint violated, reject this extension
                return null;
            }
        }
        
        // Calculate cost
        double segmentCost = calculateSegmentCost(nearest.getPoint(), newPoint, constraints);
        double newCost = nearest.getCostFromStart() + segmentCost;
        
        // Create and return the new node
        return new RRTNode(newPoint, nearest, newCost);
    }
    
    /**
     * Calculate the cost of a path segment
     */
    private double calculateSegmentCost(SpatialPoint from, SpatialPoint to, Set<RoutingConstraint> constraints) {
        // Base cost is the distance
        double cost = from.getCoordinate().distanceTo(to.getCoordinate());
        
        // Add penalties for constraint violations
        for (RoutingConstraint constraint : constraints) {
            if (!constraint.isHardConstraint() && constraint.isViolated(from, to)) {
                cost += constraint.getPenalty();
            }
        }
        
        return cost;
    }
    
    /**
     * Optimize the tree using RRT* algorithm
     */
    private void optimizeTree(List<RRTNode> tree, RRTNode newNode, Set<RoutingConstraint> constraints) {
        // 1. Find nearby nodes within radius
        double radius = calculateRRTStarRadius(tree.size());
        List<RRTNode> nearbyNodes = findNodesInRadius(tree, newNode.getPoint(), radius);
        
        // 2. Choose parent with lowest cost-to-come
        RRTNode bestParent = newNode.getParent();
        double bestCost = newNode.getCostFromStart();
        
        for (RRTNode potentialParent : nearbyNodes) {
            // Skip if it's the new node itself
            if (potentialParent.equals(newNode)) {
                continue;
            }
            
            // Check if path from potential parent is collision-free
            boolean isPathValid = true;
            for (RoutingConstraint constraint : constraints) {
                if (constraint.isHardConstraint() && 
                    constraint.isViolated(potentialParent.getPoint(), newNode.getPoint())) {
                    isPathValid = false;
                    break;
                }
            }
            
            if (!isPathValid) {
                continue;
            }
            
            // Calculate new cost
            double segmentCost = calculateSegmentCost(potentialParent.getPoint(), newNode.getPoint(), constraints);
            double newCost = potentialParent.getCostFromStart() + segmentCost;
            
            // If this gives a better path, update parent
            if (newCost < bestCost) {
                bestParent = potentialParent;
                bestCost = newCost;
            }
        }
        
        // Update parent and cost if a better one was found
        if (bestParent != newNode.getParent()) {
            newNode.setParent(bestParent);
            newNode.setCostFromStart(bestCost);
        }
        
        // 3. Rewire the tree - attempt to improve paths for nearby nodes
        for (RRTNode nearNode : nearbyNodes) {
            // Skip if it's the new node or its parent
            if (nearNode.equals(newNode) || nearNode.equals(bestParent)) {
                continue;
            }
            
            // Check if path from new node to near node is collision-free
            boolean isPathValid = true;
            for (RoutingConstraint constraint : constraints) {
                if (constraint.isHardConstraint() && 
                    constraint.isViolated(newNode.getPoint(), nearNode.getPoint())) {
                    isPathValid = false;
                    break;
                }
            }
            
            if (!isPathValid) {
                continue;
            }
            
            // Calculate potential new cost
            double segmentCost = calculateSegmentCost(newNode.getPoint(), nearNode.getPoint(), constraints);
            double potentialCost = newNode.getCostFromStart() + segmentCost;
            
            // If new path is better, rewire
            if (potentialCost < nearNode.getCostFromStart()) {
                nearNode.setParent(newNode);
                nearNode.setCostFromStart(potentialCost);
                
                // Recursively update costs for all descendants
                updateDescendantCosts(nearNode, tree);
            }
        }
    }
    
    /**
     * Calculate the optimal radius for RRT* neighbor finding
     */
    private double calculateRRTStarRadius(int treeSize) {
        // The radius shrinks as the tree grows, based on asymptotic optimal radius formula
        double dimension = 3.0; // 3D space
        double volume = config.getSearchSpacePaddingDeg() * 2 * 
                       config.getSearchSpacePaddingDeg() * 2 * 
                       config.getSearchSpaceAltPaddingFt() * 2;
        
        // gamma * (log(n)/n)^(1/d) where gamma is a constant
        double gamma = 1.5 * Math.pow(volume, 1.0/dimension);
        return gamma * Math.pow(Math.log(treeSize + 1) / (treeSize + 1), 1.0/dimension);
    }
    
    /**
     * Find all nodes within a radius of a point
     */
    private List<RRTNode> findNodesInRadius(List<RRTNode> tree, SpatialPoint point, double radius) {
        List<RRTNode> nodesInRadius = new ArrayList<>();
        
        for (RRTNode node : tree) {
            double distance = node.getPoint().getCoordinate().distanceTo(point.getCoordinate());
            if (distance <= radius) {
                nodesInRadius.add(node);
            }
        }
        
        return nodesInRadius;
    }
    
    /**
     * Update costs for all descendants of a node
     */
    private void updateDescendantCosts(RRTNode node, List<RRTNode> tree) {
        // Find all children
        List<RRTNode> children = new ArrayList<>();
        for (RRTNode potentialChild : tree) {
            if (potentialChild.getParent() == node) {
                children.add(potentialChild);
            }
        }
        
        // Update cost for each child and its descendants
        for (RRTNode child : children) {
            double segmentCost = calculateSegmentCost(node.getPoint(), child.getPoint(), Collections.emptySet());
            child.setCostFromStart(node.getCostFromStart() + segmentCost);
            
            // Recursive call for grandchildren
            updateDescendantCosts(child, tree);
        }
    }
    
    /**
     * Extract the path from root to a node
     */
    private List<SpatialPoint> extractPath(RRTNode node) {
        List<SpatialPoint> path = new ArrayList<>();
        
        // Traverse from node to root
        RRTNode current = node;
        while (current != null) {
            path.add(current.getPoint());
            current = current.getParent();
        }
        
        // Reverse to get path from root to node
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
            smoothedPath.add(path.get(farthestVisible));
            currentIndex = farthestVisible;
        }
        
        return smoothedPath;
    }
    
    /**
     * Node in the RRT tree
     */
    private static class RRTNode {
        private final SpatialPoint point;
        private RRTNode parent;
        private double costFromStart;
        
        public RRTNode(SpatialPoint point, RRTNode parent) {
            this.point = point;
            this.parent = parent;
            this.costFromStart = parent != null ? 
                parent.costFromStart + point.getCoordinate().distanceTo(parent.point.getCoordinate()) : 0;
        }
        
        public RRTNode(SpatialPoint point, RRTNode parent, double costFromStart) {
            this.point = point;
            this.parent = parent;
            this.costFromStart = costFromStart;
        }

        public SpatialPoint getPoint() {
            return point;
        }
        public RRTNode getParent() {
            return parent;
        }
        public void setParent(RRTNode parent) {
            this.parent = parent;
        }
        public double getCostFromStart() {
            return costFromStart;
        }
        public void setCostFromStart(double costFromStart) {
            this.costFromStart = costFromStart;
        }
        public boolean equals(RRTNode other) {
            return point.equals(other.point);
        }
        public int hashCode() {
            return point.hashCode();
        }
    }
}