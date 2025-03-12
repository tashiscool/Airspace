package org.tash.core.routing.lib;

import org.jgrapht.Graph;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;
import org.tash.core.SpatialElement;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.geodetic.GeodeticCalculator;
import org.tash.spatial.SpatialPoint;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectorySegment;
import org.tash.trajectory.TrajectoryType;

import java.time.ZonedDateTime;
import java.util.*;
import java.util.function.Predicate;

/**
 * Rapidly-Exploring Random Trees (RRT) path planning algorithm
 * This algorithm is particularly effective for finding paths in
 * complex environments with many obstacles
 */
public class RRTPathPlanner implements PathPlanner<SpatialPoint, TrajectorySegment> {
    
    /** The region bounds for sampling random points */
    private final GeoCoordinate regionMinBounds;
    private final GeoCoordinate regionMaxBounds;
    
    /** Maximum distance for extending the tree in nautical miles */
    private final double maxExtensionDistanceNM;
    
    /** Maximum number of iterations before giving up */
    private final int maxIterations;
    
    /** Random number generator */
    private final Random random;
    
    /** Edge factory for creating new edges */
    private final EdgeFactory<SpatialPoint, TrajectorySegment> edgeFactory;
    
    /** Goal bias probability (chance of sampling directly toward the goal) */
    private final double goalBias;
    
    /** Distance threshold for reaching the goal in nautical miles */
    private final double goalThresholdNM;
    
    /** Path smoothing passes to perform */
    private final int pathSmoothingPasses;
    
    /**
     * Constructor with custom parameters
     * 
     * @param regionMinBounds Minimum bounds of the region
     * @param regionMaxBounds Maximum bounds of the region
     * @param maxExtensionDistanceNM Maximum distance for extending the tree
     * @param maxIterations Maximum number of iterations
     * @param edgeFactory Edge factory for creating new edges
     * @param goalBias Goal bias probability (0.0 to 1.0)
     * @param goalThresholdNM Distance threshold for reaching the goal
     * @param pathSmoothingPasses Number of path smoothing passes
     */
    public RRTPathPlanner(GeoCoordinate regionMinBounds, GeoCoordinate regionMaxBounds,
                        double maxExtensionDistanceNM, int maxIterations,
                        EdgeFactory<SpatialPoint, TrajectorySegment> edgeFactory,
                        double goalBias, double goalThresholdNM, int pathSmoothingPasses) {
        this.regionMinBounds = regionMinBounds;
        this.regionMaxBounds = regionMaxBounds;
        this.maxExtensionDistanceNM = maxExtensionDistanceNM;
        this.maxIterations = maxIterations;
        this.random = new Random();
        this.edgeFactory = edgeFactory;
        this.goalBias = goalBias;
        this.goalThresholdNM = goalThresholdNM;
        this.pathSmoothingPasses = pathSmoothingPasses;
    }
    
    /**
     * Constructor with default parameters
     * 
     * @param regionMinBounds Minimum bounds of the region
     * @param regionMaxBounds Maximum bounds of the region
     * @param edgeFactory Edge factory for creating new edges
     */
    public RRTPathPlanner(GeoCoordinate regionMinBounds, GeoCoordinate regionMaxBounds,
                        EdgeFactory<SpatialPoint, TrajectorySegment> edgeFactory) {
        this(regionMinBounds, regionMaxBounds, 20.0, 5000, edgeFactory, 0.05, 1.0, 3);
    }
    
    @Override
    public List<TrajectorySegment> findPath(SpatialPoint start, SpatialPoint goal) {
        // This is an unconstrained case, so we call the constrained version with no obstacles
        return findPath(start, goal, Collections.emptySet());
    }
    
    @Override
    public List<TrajectorySegment> findPath(SpatialPoint start, SpatialPoint goal, 
                                          Set<? extends SpatialElement> obstacles) {
        // Initialize the tree with the start node
        RRTNode rootNode = new RRTNode(start, null, null);
        List<RRTNode> tree = new ArrayList<>();
        tree.add(rootNode);
        
        RRTNode goalNode = null;
        boolean reachedGoal = false;
        
        // Main RRT loop
        for (int i = 0; i < maxIterations; i++) {
            // Sample a random point
            SpatialPoint randomPoint = sampleRandomPoint(goal);
            
            // Find the nearest node in the tree
            RRTNode nearestNode = findNearestNode(tree, randomPoint);
            
            // Extend the tree toward the random point
            RRTNode newNode = extendTree(nearestNode, randomPoint, obstacles);
            
            if (newNode != null) {
                tree.add(newNode);
                
                // Check if we've reached the goal
                double distanceToGoal = GeodeticCalculator.vincentyDistance(
                    newNode.getPoint().getCoordinate(), goal.getCoordinate());
                
                if (distanceToGoal <= goalThresholdNM) {
                    // Create the final edge to the goal
                    TrajectorySegment finalEdge = edgeFactory.createEdge(newNode.getPoint(), goal);
                    
                    // Check if the final edge is obstacle-free
                    boolean obstacleFree = true;
                    for (SpatialElement obstacle : obstacles) {
                        if (finalEdge.intersects(obstacle)) {
                            obstacleFree = false;
                            break;
                        }
                    }
                    
                    if (obstacleFree) {
                        goalNode = new RRTNode(goal, newNode, finalEdge);
                        tree.add(goalNode);
                        reachedGoal = true;
                        break;
                    }
                }
            }
        }
        
        if (!reachedGoal || goalNode == null) {
            return new ArrayList<>(); // No path found
        }
        
        // Reconstruct the path
        List<TrajectorySegment> path = extractPath(rootNode, goalNode);
        
        // Apply path smoothing if requested
        if (pathSmoothingPasses > 0) {
            path = smoothPath(path, obstacles);
        }
        
        return path;
    }
    
    @Override
    public List<TrajectorySegment> findPathWithConstraints(SpatialPoint start, SpatialPoint goal, 
                                                        Predicate<TrajectorySegment> constraints) {
        // Initialize the tree with the start node
        RRTNode rootNode = new RRTNode(start, null, null);
        List<RRTNode> tree = new ArrayList<>();
        tree.add(rootNode);
        
        RRTNode goalNode = null;
        boolean reachedGoal = false;
        
        // Main RRT loop
        for (int i = 0; i < maxIterations; i++) {
            // Sample a random point
            SpatialPoint randomPoint = sampleRandomPoint(goal);
            
            // Find the nearest node in the tree
            RRTNode nearestNode = findNearestNode(tree, randomPoint);
            
            // Extend the tree toward the random point
            RRTNode newNode = extendTreeWithConstraints(nearestNode, randomPoint, constraints);
            
            if (newNode != null) {
                tree.add(newNode);
                
                // Check if we've reached the goal
                double distanceToGoal = GeodeticCalculator.vincentyDistance(
                    newNode.getPoint().getCoordinate(), goal.getCoordinate());
                
                if (distanceToGoal <= goalThresholdNM) {
                    // Create the final edge to the goal
                    TrajectorySegment finalEdge = edgeFactory.createEdge(newNode.getPoint(), goal);
                    
                    // Check if the final edge satisfies the constraints
                    if (constraints.test(finalEdge)) {
                        goalNode = new RRTNode(goal, newNode, finalEdge);
                        tree.add(goalNode);
                        reachedGoal = true;
                        break;
                    }
                }
            }
        }
        
        if (!reachedGoal || goalNode == null) {
            return new ArrayList<>(); // No path found
        }
        
        // Reconstruct the path
        List<TrajectorySegment> path = extractPath(rootNode, goalNode);
        
        // Apply path smoothing if requested
        if (pathSmoothingPasses > 0) {
            path = smoothPathWithConstraints(path, constraints);
        }
        
        return path;
    }
    
    /**
     * Sample a random point within the region bounds
     * Incorporates goal bias to sometimes sample directly toward the goal
     * 
     * @param goal The goal point
     * @return A random spatial point
     */
    private SpatialPoint sampleRandomPoint(SpatialPoint goal) {
        // Check for goal bias
        if (random.nextDouble() < goalBias) {
            return goal;
        }
        
        // Sample a random point within the region bounds
        double lat = regionMinBounds.getLatitude() + 
                    random.nextDouble() * (regionMaxBounds.getLatitude() - regionMinBounds.getLatitude());
        double lon = regionMinBounds.getLongitude() + 
                    random.nextDouble() * (regionMaxBounds.getLongitude() - regionMinBounds.getLongitude());
        double alt = regionMinBounds.getAltitude() + 
                    random.nextDouble() * (regionMaxBounds.getAltitude() - regionMinBounds.getAltitude());
        
        GeoCoordinate randomCoord = GeoCoordinate.builder()
            .latitude(lat)
            .longitude(lon)
            .altitude(alt)
            .build();
        
        return SpatialPoint.builder()
            .id("RRT-" + UUID.randomUUID().toString())
            .coordinate(randomCoord)
            .build();
    }
    
    /**
     * Find the nearest node in the tree to a point
     * 
     * @param tree The RRT tree
     * @param point The point to find the nearest node to
     * @return The nearest node
     */
    private RRTNode findNearestNode(List<RRTNode> tree, SpatialPoint point) {
        RRTNode nearest = null;
        double minDistance = Double.POSITIVE_INFINITY;
        
        for (RRTNode node : tree) {
            double distance = GeodeticCalculator.vincentyDistance(
                node.getPoint().getCoordinate(), point.getCoordinate());
            
            if (distance < minDistance) {
                minDistance = distance;
                nearest = node;
            }
        }
        
        return nearest;
    }
    
    /**
     * Extend the tree toward a random point
     * 
     * @param nearestNode The nearest node in the tree
     * @param randomPoint The random point to extend towards
     * @param obstacles Set of obstacles to avoid
     * @return The new node, or null if extension failed
     */
    private RRTNode extendTree(RRTNode nearestNode, SpatialPoint randomPoint, 
                              Set<? extends SpatialElement> obstacles) {
        // Calculate direction from nearest to random
        GeoCoordinate nearestCoord = nearestNode.getPoint().getCoordinate();
        GeoCoordinate randomCoord = randomPoint.getCoordinate();
        
        // Calculate distance and bearing
        double distance = GeodeticCalculator.vincentyDistance(nearestCoord, randomCoord);
        double bearing = GeodeticCalculator.initialBearing(nearestCoord, randomCoord);
        
        // Limit the extension distance
        distance = Math.min(distance, maxExtensionDistanceNM);
        
        // Calculate the new point
        GeoCoordinate newCoord = GeodeticCalculator.destinationPoint(nearestCoord, distance, bearing);
        
        // Create the new point
        SpatialPoint newPoint = SpatialPoint.builder()
            .id("RRT-" + UUID.randomUUID().toString())
            .coordinate(newCoord)
            .build();
        
        // Create the edge
        TrajectorySegment edge = edgeFactory.createEdge(nearestNode.getPoint(), newPoint);
        
        // Check if the edge intersects any obstacles
        for (SpatialElement obstacle : obstacles) {
            if (edge.intersects(obstacle)) {
                return null; // Extension failed
            }
        }
        
        // Create and return the new node
        return new RRTNode(newPoint, nearestNode, edge);
    }
    
    /**
     * Extend the tree toward a random point with constraints
     * 
     * @param nearestNode The nearest node in the tree
     * @param randomPoint The random point to extend towards
     * @param constraints Predicate that returns true for valid edges
     * @return The new node, or null if extension failed
     */
    private RRTNode extendTreeWithConstraints(RRTNode nearestNode, SpatialPoint randomPoint, 
                                           Predicate<TrajectorySegment> constraints) {
        // Calculate direction from nearest to random
        GeoCoordinate nearestCoord = nearestNode.getPoint().getCoordinate();
        GeoCoordinate randomCoord = randomPoint.getCoordinate();
        
        // Calculate distance and bearing
        double distance = GeodeticCalculator.vincentyDistance(nearestCoord, randomCoord);
        double bearing = GeodeticCalculator.initialBearing(nearestCoord, randomCoord);
        
        // Limit the extension distance
        distance = Math.min(distance, maxExtensionDistanceNM);
        
        // Calculate the new point
        GeoCoordinate newCoord = GeodeticCalculator.destinationPoint(nearestCoord, distance, bearing);
        
        // Create the new point
        SpatialPoint newPoint = SpatialPoint.builder()
            .id("RRT-" + UUID.randomUUID().toString())
            .coordinate(newCoord)
            .build();
        
        // Create the edge
        TrajectorySegment edge = edgeFactory.createEdge(nearestNode.getPoint(), newPoint);
        
        // Check if the edge satisfies the constraints
        if (!constraints.test(edge)) {
            return null; // Extension failed
        }
        
        // Create and return the new node
        return new RRTNode(newPoint, nearestNode, edge);
    }
    
    /**
     * Extract the path from the RRT tree
     * 
     * @param rootNode The root node of the tree
     * @param goalNode The goal node
     * @return The list of trajectory segments forming the path
     */
    private List<TrajectorySegment> extractPath(RRTNode rootNode, RRTNode goalNode) {
        List<TrajectorySegment> path = new ArrayList<>();
        RRTNode currentNode = goalNode;
        
        while (currentNode != rootNode) {
            if (currentNode.getParent() == null) {
                break; // Sanity check - should not happen
            }
            
            path.add(0, currentNode.getEdge());
            currentNode = currentNode.getParent();
        }
        
        return path;
    }
    
    /**
     * Smooth the path by removing unnecessary waypoints
     * 
     * @param path The original path
     * @param obstacles Set of obstacles to avoid
     * @return The smoothed path
     */
    private List<TrajectorySegment> smoothPath(List<TrajectorySegment> path, 
                                            Set<? extends SpatialElement> obstacles) {
        if (path.size() <= 2) {
            return path; // No smoothing needed for very short paths
        }
        
        // Create a new path
        List<TrajectorySegment> smoothedPath = new ArrayList<>(path);
        
        // Apply multiple passes of smoothing
        for (int pass = 0; pass < pathSmoothingPasses; pass++) {
            boolean changes = false;
            
            // Start with i at 0 and consider removing intermediate nodes
            for (int i = 0; i < smoothedPath.size() - 2; i++) {
                // Try to create a direct edge from i to i+2
                SpatialPoint start = smoothedPath.get(i).getSource();
                SpatialPoint end = smoothedPath.get(i + 1).getTarget();
                
                TrajectorySegment directEdge = edgeFactory.createEdge(start, end);
                
                // Check if direct edge intersects any obstacles
                boolean edgeValid = true;
                for (SpatialElement obstacle : obstacles) {
                    if (directEdge.intersects(obstacle)) {
                        edgeValid = false;
                        break;
                    }
                }
                
                if (edgeValid) {
                    // Replace two segments with one direct segment
                    smoothedPath.set(i, directEdge);
                    smoothedPath.remove(i + 1);
                    changes = true;
                    i--; // Recheck this index since we removed an element
                }
            }
            
            if (!changes) {
                break; // No more improvements possible
            }
        }
        
        return smoothedPath;
    }
    
    /**
     * Smooth the path by removing unnecessary waypoints, with constraints
     * 
     * @param path The original path
     * @param constraints Predicate that returns true for valid edges
     * @return The smoothed path
     */
    private List<TrajectorySegment> smoothPathWithConstraints(List<TrajectorySegment> path, 
                                                         Predicate<TrajectorySegment> constraints) {
        if (path.size() <= 2) {
            return path; // No smoothing needed for very short paths
        }
        
        // Create a new path
        List<TrajectorySegment> smoothedPath = new ArrayList<>(path);
        
        // Apply multiple passes of smoothing
        for (int pass = 0; pass < pathSmoothingPasses; pass++) {
            boolean changes = false;
            
            // Start with i at 0 and consider removing intermediate nodes
            for (int i = 0; i < smoothedPath.size() - 2; i++) {
                // Try to create a direct edge from i to i+2
                SpatialPoint start = smoothedPath.get(i).getSource();
                SpatialPoint end = smoothedPath.get(i + 1).getTarget();
                
                TrajectorySegment directEdge = edgeFactory.createEdge(start, end);
                
                // Check if direct edge satisfies constraints
                if (constraints.test(directEdge)) {
                    // Replace two segments with one direct segment
                    smoothedPath.set(i, directEdge);
                    smoothedPath.remove(i + 1);
                    changes = true;
                    i--; // Recheck this index since we removed an element
                }
            }
            
            if (!changes) {
                break; // No more improvements possible
            }
        }
        
        return smoothedPath;
    }
    
    /**
     * Node class for the RRT tree
     */
    private static class RRTNode {
        private final SpatialPoint point;
        private final RRTNode parent;
        private final TrajectorySegment edge; // Edge from parent to this node
        
        public RRTNode(SpatialPoint point, RRTNode parent, TrajectorySegment edge) {
            this.point = point;
            this.parent = parent;
            this.edge = edge;
        }
        
        public SpatialPoint getPoint() {
            return point;
        }
        
        public RRTNode getParent() {
            return parent;
        }
        
        public TrajectorySegment getEdge() {
            return edge;
        }
    }
    
    /**
     * Create a graph from the RRT tree
     * Useful for visualization and integration with other graph algorithms
     * 
     * @param tree The RRT tree
     * @return A graph representation of the tree
     */
    public static Graph<SpatialPoint, TrajectorySegment> createGraphFromRRTTree(List<RRTNode> tree) {
        Graph<SpatialPoint, TrajectorySegment> graph = 
            new SimpleDirectedWeightedGraph<>(TrajectorySegment.class);
        
        // Add all nodes to the graph
        for (RRTNode node : tree) {
            graph.addVertex(node.getPoint());
        }
        
        // Add all edges
        for (RRTNode node : tree) {
            if (node.getParent() != null && node.getEdge() != null) {
                graph.addEdge(node.getParent().getPoint(), node.getPoint(), node.getEdge());
                
                // Set edge weight as the distance
                double distance = GeodeticCalculator.vincentyDistance(
                    node.getParent().getPoint().getCoordinate(), 
                    node.getPoint().getCoordinate());
                
                graph.setEdgeWeight(node.getEdge(), distance);
            }
        }
        
        return graph;
    }
}