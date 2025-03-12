package org.tash.core.routing.lib;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.interfaces.ShortestPathAlgorithm;
import org.jgrapht.alg.shortestpath.AStarShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;
import org.tash.core.SpatialElement;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.geodetic.GeodeticCalculator;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectorySegment;
import org.tash.trajectory.TrajectoryType;

import java.time.ZonedDateTime;
import java.util.*;
import java.util.function.Predicate;
import java.util.stream.Collectors;

/**
 * Implementation of A* path planning algorithm
 * Uses JGraphT A* implementation with custom heuristics
 */
public class AStarPathPlanner implements MultiPathPlanner<SpatialPoint, TrajectorySegment> {

    /** The graph containing vertices (waypoints) and edges (trajectory segments) */
    private final Graph<SpatialPoint, TrajectorySegment> graph;

    /** Cost function for edges */
    private final CostFunction<SpatialPoint, TrajectorySegment> costFunction;

    /** Heuristic function for A* */
    private final HeuristicFunction<SpatialPoint> heuristicFunction;

    /** Edge factory for creating new edges */
    private final EdgeFactory<SpatialPoint, TrajectorySegment> edgeFactory;

    /** Time to use for segment creation */
    private final ZonedDateTime baseTime;

    /** Default speed in knots to use for time estimation */
    private final double defaultSpeedKnots;

    /** Obstacle clearance distance in nautical miles */
    private final double obstacleClearanceNM;

    /** Flag indicating whether to use great circle paths */
    private final boolean useGreatCircle;

    /** Maximum number of iterations to run before giving up */
    private final int maxIterations;

    /**
     * Constructor with default heuristic and cost functions
     *
     * @param graph The graph to use for path planning
     * @param baseTime Base time to use for segment creation
     * @param defaultSpeedKnots Default speed in knots for time estimation
     */
    public AStarPathPlanner(Graph<SpatialPoint, TrajectorySegment> graph,
                            ZonedDateTime baseTime,
                            double defaultSpeedKnots) {
        this(graph, baseTime, defaultSpeedKnots,
                createDefaultCostFunction(),
                createDefaultHeuristicFunction(),
                createDefaultEdgeFactory(baseTime, defaultSpeedKnots));
    }

    /**
     * Constructor with custom heuristic and cost functions
     *
     * @param graph The graph to use for path planning
     * @param baseTime Base time to use for segment creation
     * @param defaultSpeedKnots Default speed in knots for time estimation
     * @param costFunction Cost function to use
     * @param heuristicFunction Heuristic function to use
     * @param edgeFactory Edge factory to use
     */
    public AStarPathPlanner(Graph<SpatialPoint, TrajectorySegment> graph,
                            ZonedDateTime baseTime,
                            double defaultSpeedKnots,
                            CostFunction<SpatialPoint, TrajectorySegment> costFunction,
                            HeuristicFunction<SpatialPoint> heuristicFunction,
                            EdgeFactory<SpatialPoint, TrajectorySegment> edgeFactory) {
        this.graph = graph;
        this.baseTime = baseTime;
        this.defaultSpeedKnots = defaultSpeedKnots;
        this.costFunction = costFunction;
        this.heuristicFunction = heuristicFunction;
        this.edgeFactory = edgeFactory;
        this.obstacleClearanceNM = 5.0; // 5 nautical miles default clearance
        this.useGreatCircle = true;
        this.maxIterations = 10000;
    }

    /**
     * Create a default cost function based on distance
     *
     * @return Default cost function
     */
    public static CostFunction<SpatialPoint, TrajectorySegment> createDefaultCostFunction() {
        return new CostFunction<SpatialPoint, TrajectorySegment>() {
            @Override
            public double getCost(TrajectorySegment edge) {
                // Use great circle distance as cost
                GeoCoordinate start = edge.getSource().getCoordinate();
                GeoCoordinate end = edge.getTarget().getCoordinate();
                return GeodeticCalculator.vincentyDistance(start, end);
            }

            @Override
            public double getCost(SpatialPoint from, SpatialPoint to) {
                // Direct distance between points
                return GeodeticCalculator.vincentyDistance(
                        from.getCoordinate(), to.getCoordinate());
            }
        };
    }

    /**
     * Create a default heuristic function based on direct distance
     *
     * @return Default heuristic function
     */
    public static HeuristicFunction<SpatialPoint> createDefaultHeuristicFunction() {
        return (current, goal) -> {
            // Direct distance to goal as heuristic (admissible)
            return GeodeticCalculator.vincentyDistance(
                    current.getCoordinate(), goal.getCoordinate());
        };
    }

    /**
     * Create a default edge factory that creates linear trajectory segments
     *
     * @param baseTime Base time to use
     * @param speedKnots Speed in knots for time estimation
     * @return Default edge factory
     */
    public static EdgeFactory<SpatialPoint, TrajectorySegment> createDefaultEdgeFactory(
            ZonedDateTime baseTime, double speedKnots) {
        return (from, to) -> {
            // Calculate distance to estimate time
            double distanceNM = GeodeticCalculator.vincentyDistance(
                    from.getCoordinate(), to.getCoordinate());

            // Calculate time in hours, then convert to minutes
            double timeHours = distanceNM / speedKnots;
            long timeMinutes = (long) (timeHours * 60);

            // Create a linear trajectory segment
            LinearTrajectorySegment linear = new LinearTrajectorySegment();
            linear.setStartTime(baseTime);
            return linear;
        };
    }

    @Override
    public List<TrajectorySegment> findPath(SpatialPoint start, SpatialPoint goal) {
        // Create A* algorithm with our heuristic
        AStarShortestPath<SpatialPoint, TrajectorySegment> astar =
                new AStarShortestPath<>(graph,
                        (vertex, goal1) -> heuristicFunction.estimate(vertex, goal1));

        // Calculate the path
        GraphPath<SpatialPoint, TrajectorySegment> path = astar.getPath(start, goal);

        return path != null ? path.getEdgeList() : new ArrayList<>();
    }

    @Override
    public List<TrajectorySegment> findPath(SpatialPoint start, SpatialPoint goal,
                                            Set<? extends SpatialElement> obstacles) {
        // Create a graph copy to work with
        Graph<SpatialPoint, TrajectorySegment> workGraph =
                createWorkingGraph(graph, obstacles, obstacleClearanceNM);

        // Make sure start and goal are in the graph
        addVerticesToGraph(workGraph, start, goal);

        // Connect start and goal to nearby vertices
        connectVertexToGraph(workGraph, start, 50.0);
        connectVertexToGraph(workGraph, goal, 50.0);

        // Create A* algorithm with our heuristic
        AStarShortestPath<SpatialPoint, TrajectorySegment> astar =
                new AStarShortestPath<>(workGraph,
                        (vertex, goal1) -> heuristicFunction.estimate(vertex, goal1));

        // Calculate the path
        GraphPath<SpatialPoint, TrajectorySegment> path = astar.getPath(start, goal);

        return path != null ? path.getEdgeList() : new ArrayList<>();
    }

    @Override
    public List<TrajectorySegment> findPathWithConstraints(SpatialPoint start, SpatialPoint goal,
                                                           Predicate<TrajectorySegment> constraints) {
        // Create a graph copy to work with
        Graph<SpatialPoint, TrajectorySegment> workGraph =
                createWorkingGraph(graph, constraints);

        // Make sure start and goal are in the graph
        addVerticesToGraph(workGraph, start, goal);

        // Connect start and goal to nearby vertices with constraints
        connectVertexToGraph(workGraph, start, 50.0, constraints);
        connectVertexToGraph(workGraph, goal, 50.0, constraints);

        // Create A* algorithm with our heuristic
        AStarShortestPath<SpatialPoint, TrajectorySegment> astar =
                new AStarShortestPath<>(workGraph,
                        (vertex, goal1) -> heuristicFunction.estimate(vertex, goal1));

        // Calculate the path
        GraphPath<SpatialPoint, TrajectorySegment> path = astar.getPath(start, goal);

        return path != null ? path.getEdgeList() : new ArrayList<>();
    }

    @Override
    public List<List<TrajectorySegment>> findMultiplePaths(SpatialPoint start, SpatialPoint goal, int numPaths) {
        if (numPaths <= 0) {
            return new ArrayList<>();
        }

        List<List<TrajectorySegment>> paths = new ArrayList<>();

        // Find the first (shortest) path
        List<TrajectorySegment> shortestPath = findPath(start, goal);
        if (shortestPath.isEmpty()) {
            return paths; // No path found
        }

        paths.add(shortestPath);

        // For each additional path, remove an edge from previous paths and find a new path
        for (int i = 1; i < numPaths; i++) {
            // Create a graph copy to work with
            Graph<SpatialPoint, TrajectorySegment> workGraph =
                    createWorkingGraphCopy(graph);

            // Remove edges from previous paths
            for (List<TrajectorySegment> prevPath : paths) {
                // Choose a random segment to remove from each previous path
                if (!prevPath.isEmpty()) {
                    int segmentToRemove = new Random().nextInt(prevPath.size());
                    TrajectorySegment segment = prevPath.get(segmentToRemove);
                    workGraph.removeEdge(segment);
                }
            }

            // Create A* algorithm with our heuristic
            AStarShortestPath<SpatialPoint, TrajectorySegment> astar =
                    new AStarShortestPath<>(workGraph,
                            (vertex, goal1) -> heuristicFunction.estimate(vertex, goal1));

            // Calculate the path
            GraphPath<SpatialPoint, TrajectorySegment> path = astar.getPath(start, goal);

            if (path != null && !path.getEdgeList().isEmpty()) {
                paths.add(path.getEdgeList());
            } else {
                // No more paths found
                break;
            }
        }

        return paths;
    }

    @Override
    public List<List<TrajectorySegment>> findMultiplePaths(SpatialPoint start, SpatialPoint goal,
                                                           Set<? extends SpatialElement> obstacles,
                                                           int numPaths) {
        if (numPaths <= 0) {
            return new ArrayList<>();
        }

        // Create a graph copy to work with
        Graph<SpatialPoint, TrajectorySegment> workGraph =
                createWorkingGraph(graph, obstacles, obstacleClearanceNM);

        // Make sure start and goal are in the graph
        addVerticesToGraph(workGraph, start, goal);

        // Connect start and goal to nearby vertices
        connectVertexToGraph(workGraph, start, 50.0);
        connectVertexToGraph(workGraph, goal, 50.0);

        List<List<TrajectorySegment>> paths = new ArrayList<>();

        // Create a copy for each path finding
        Graph<SpatialPoint, TrajectorySegment> workGraphCopy =
                createWorkingGraphCopy(workGraph);

        // Find the first (shortest) path
        AStarShortestPath<SpatialPoint, TrajectorySegment> astar =
                new AStarShortestPath<>(workGraphCopy,
                        (vertex, goal1) -> heuristicFunction.estimate(vertex, goal1));

        GraphPath<SpatialPoint, TrajectorySegment> path = astar.getPath(start, goal);

        if (path == null || path.getEdgeList().isEmpty()) {
            return paths; // No path found
        }

        paths.add(path.getEdgeList());

        // For each additional path, remove edges from previous paths and find a new path
        for (int i = 1; i < numPaths; i++) {
            // Create a new graph copy
            workGraphCopy = createWorkingGraphCopy(workGraph);

            // Remove edges from previous paths
            for (List<TrajectorySegment> prevPath : paths) {
                // Choose a random segment to remove from each previous path
                if (!prevPath.isEmpty()) {
                    int segmentToRemove = new Random().nextInt(prevPath.size());
                    TrajectorySegment segment = prevPath.get(segmentToRemove);
                    workGraphCopy.removeEdge(segment);
                }
            }

            // Create A* algorithm with our heuristic
            astar = new AStarShortestPath<>(workGraphCopy,
                    (vertex, goal1) -> heuristicFunction.estimate(vertex, goal1));

            // Calculate the path
            path = astar.getPath(start, goal);

            if (path != null && !path.getEdgeList().isEmpty()) {
                paths.add(path.getEdgeList());
            } else {
                // No more paths found
                break;
            }
        }

        return paths;
    }

    @Override
    public List<List<TrajectorySegment>> findMultiplePathsWithConstraints(SpatialPoint start, SpatialPoint goal,
                                                                          Predicate<TrajectorySegment> constraints,
                                                                          int numPaths) {
        if (numPaths <= 0) {
            return new ArrayList<>();
        }

        // Create a graph copy to work with
        Graph<SpatialPoint, TrajectorySegment> workGraph =
                createWorkingGraph(graph, constraints);

        // Make sure start and goal are in the graph
        addVerticesToGraph(workGraph, start, goal);

        // Connect start and goal to nearby vertices with constraints
        connectVertexToGraph(workGraph, start, 50.0, constraints);
        connectVertexToGraph(workGraph, goal, 50.0, constraints);

        List<List<TrajectorySegment>> paths = new ArrayList<>();

        // Create a copy for each path finding
        Graph<SpatialPoint, TrajectorySegment> workGraphCopy =
                createWorkingGraphCopy(workGraph);

        // Find the first (shortest) path
        AStarShortestPath<SpatialPoint, TrajectorySegment> astar =
                new AStarShortestPath<>(workGraphCopy,
                        (vertex, goal1) -> heuristicFunction.estimate(vertex, goal1));

        GraphPath<SpatialPoint, TrajectorySegment> path = astar.getPath(start, goal);

        if (path == null || path.getEdgeList().isEmpty()) {
            return paths; // No path found
        }

        paths.add(path.getEdgeList());

        // For each additional path, remove edges from previous paths and find a new path
        for (int i = 1; i < numPaths; i++) {
            // Create a new graph copy
            workGraphCopy = createWorkingGraphCopy(workGraph);

            // Remove some edges from previous paths to find diverse alternatives
            for (List<TrajectorySegment> prevPath : paths) {
                // Choose a random segment to remove from each previous path
                if (!prevPath.isEmpty()) {
                    int segmentToRemove = new Random().nextInt(prevPath.size());
                    TrajectorySegment segment = prevPath.get(segmentToRemove);
                    workGraphCopy.removeEdge(segment);
                }
            }

            // Create A* algorithm with our heuristic
            astar = new AStarShortestPath<>(workGraphCopy,
                    (vertex, goal1) -> heuristicFunction.estimate(vertex, goal1));

            // Calculate the path
            path = astar.getPath(start, goal);

            if (path != null && !path.getEdgeList().isEmpty()) {
                paths.add(path.getEdgeList());
            } else {
                // No more paths found
                break;
            }
        }

        return paths;
    }

    /**
     * Create a working copy of the graph
     *
     * @param sourceGraph Source graph to copy
     * @return A copy of the graph
     */
    private Graph<SpatialPoint, TrajectorySegment> createWorkingGraphCopy(
            Graph<SpatialPoint, TrajectorySegment> sourceGraph) {

        // Create a new graph of the same type
        Graph<SpatialPoint, TrajectorySegment> workGraph =
                new SimpleDirectedWeightedGraph<>(TrajectorySegment.class);

        // Copy all vertices
        for (SpatialPoint vertex : sourceGraph.vertexSet()) {
            workGraph.addVertex(vertex);
        }

        // Copy all edges
        for (TrajectorySegment edge : sourceGraph.edgeSet()) {
            SpatialPoint source = sourceGraph.getEdgeSource(edge);
            SpatialPoint target = sourceGraph.getEdgeTarget(edge);

            workGraph.addEdge(source, target, edge);
            workGraph.setEdgeWeight(edge, costFunction.getCost(edge));
        }

        return workGraph;
    }

    /**
     * Create a working graph avoiding obstacles
     *
     * @param sourceGraph Source graph to copy
     * @param obstacles Set of obstacles to avoid
     * @param clearanceNM Clearance distance in nautical miles
     * @return A graph with no edges intersecting obstacles
     */
    private Graph<SpatialPoint, TrajectorySegment> createWorkingGraph(
            Graph<SpatialPoint, TrajectorySegment> sourceGraph,
            Set<? extends SpatialElement> obstacles,
            double clearanceNM) {

        // Create a new graph of the same type
        Graph<SpatialPoint, TrajectorySegment> workGraph =
                new SimpleDirectedWeightedGraph<>(TrajectorySegment.class);

        // Copy all vertices
        for (SpatialPoint vertex : sourceGraph.vertexSet()) {
            workGraph.addVertex(vertex);
        }

        // Copy only edges that don't intersect obstacles
        for (TrajectorySegment edge : sourceGraph.edgeSet()) {
            SpatialPoint source = sourceGraph.getEdgeSource(edge);
            SpatialPoint target = sourceGraph.getEdgeTarget(edge);

            boolean intersectsObstacle = false;
            for (SpatialElement obstacle : obstacles) {
                if (edge.intersects(obstacle)) {
                    intersectsObstacle = true;
                    break;
                }
            }

            if (!intersectsObstacle) {
                workGraph.addEdge(source, target, edge);
                workGraph.setEdgeWeight(edge, costFunction.getCost(edge));
            }
        }

        return workGraph;
    }

    /**
     * Create a working graph with constraints
     *
     * @param sourceGraph Source graph to copy
     * @param constraints Predicate that returns true for valid edges
     * @return A graph with only edges that satisfy constraints
     */
    private Graph<SpatialPoint, TrajectorySegment> createWorkingGraph(
            Graph<SpatialPoint, TrajectorySegment> sourceGraph,
            Predicate<TrajectorySegment> constraints) {

        // Create a new graph of the same type
        Graph<SpatialPoint, TrajectorySegment> workGraph =
                new SimpleDirectedWeightedGraph<>(TrajectorySegment.class);

        // Copy all vertices
        for (SpatialPoint vertex : sourceGraph.vertexSet()) {
            workGraph.addVertex(vertex);
        }

        // Copy only edges that satisfy constraints
        for (TrajectorySegment edge : sourceGraph.edgeSet()) {
            SpatialPoint source = sourceGraph.getEdgeSource(edge);
            SpatialPoint target = sourceGraph.getEdgeTarget(edge);

            if (constraints.test(edge)) {
                workGraph.addEdge(source, target, edge);
                workGraph.setEdgeWeight(edge, costFunction.getCost(edge));
            }
        }

        return workGraph;
    }

    /**
     * Add vertices to the graph
     *
     * @param graph Graph to add vertices to
     * @param vertices Vertices to add
     */
    private void addVerticesToGraph(Graph<SpatialPoint, TrajectorySegment> graph,
                                    SpatialPoint... vertices) {
        for (SpatialPoint vertex : vertices) {
            graph.addVertex(vertex);
        }
    }

    /**
     * Connect a vertex to nearby vertices in the graph
     *
     * @param graph Graph to connect to
     * @param vertex Vertex to connect
     * @param maxDistanceNM Maximum distance to connect to in nautical miles
     */
    private void connectVertexToGraph(Graph<SpatialPoint, TrajectorySegment> graph,
                                      SpatialPoint vertex,
                                      double maxDistanceNM) {
        // Connect to all vertices within the max distance
        for (SpatialPoint target : graph.vertexSet()) {
            if (target.equals(vertex)) {
                continue; // Skip self
            }

            double distance = GeodeticCalculator.vincentyDistance(
                    vertex.getCoordinate(), target.getCoordinate());

            if (distance <= maxDistanceNM) {
                TrajectorySegment edge = edgeFactory.createEdge(vertex, target);
                graph.addEdge(vertex, target, edge);
                graph.setEdgeWeight(edge, costFunction.getCost(edge));

                // Add reverse edge too for bidirectional connectivity
                TrajectorySegment reverseEdge = edgeFactory.createEdge(target, vertex);
                graph.addEdge(target, vertex, reverseEdge);
                graph.setEdgeWeight(reverseEdge, costFunction.getCost(reverseEdge));
            }
        }
    }

    /**
     * Connect a vertex to nearby vertices in the graph with constraints
     *
     * @param graph Graph to connect to
     * @param vertex Vertex to connect
     * @param maxDistanceNM Maximum distance to connect to in nautical miles
     * @param constraints Predicate that returns true for valid edges
     */
    private void connectVertexToGraph(Graph<SpatialPoint, TrajectorySegment> graph,
                                      SpatialPoint vertex,
                                      double maxDistanceNM,
                                      Predicate<TrajectorySegment> constraints) {
        // Connect to all vertices within the max distance that satisfy constraints
        for (SpatialPoint target : graph.vertexSet()) {
            if (target.equals(vertex)) {
                continue; // Skip self
            }

            double distance = GeodeticCalculator.vincentyDistance(
                    vertex.getCoordinate(), target.getCoordinate());

            if (distance <= maxDistanceNM) {
                TrajectorySegment edge = edgeFactory.createEdge(vertex, target);

                if (constraints.test(edge)) {
                    graph.addEdge(vertex, target, edge);
                    graph.setEdgeWeight(edge, costFunction.getCost(edge));

                    // Add reverse edge too for bidirectional connectivity if it satisfies constraints
                    TrajectorySegment reverseEdge = edgeFactory.createEdge(target, vertex);

                    if (constraints.test(reverseEdge)) {
                        graph.addEdge(target, vertex, reverseEdge);
                        graph.setEdgeWeight(reverseEdge, costFunction.getCost(reverseEdge));
                    }
                }
            }
        }
    }

    /**
     * Create a weighted graph from a set of points
     *
     * @param points Set of points to use as vertices
     * @param maxConnectDistanceNM Maximum distance to connect points in nautical miles
     * @param directedGraph Whether to create a directed graph (true) or undirected (false)
     * @return The constructed graph
     */
    public static Graph<SpatialPoint, TrajectorySegment> createGraphFromPoints(
            Set<SpatialPoint> points,
            double maxConnectDistanceNM,
            boolean directedGraph,
            EdgeFactory<SpatialPoint, TrajectorySegment> edgeFactory,
            CostFunction<SpatialPoint, TrajectorySegment> costFunction) {

        // Create a new graph
        Graph<SpatialPoint, TrajectorySegment> graph =
                new SimpleDirectedWeightedGraph<>(TrajectorySegment.class);

        // Add all points as vertices
        for (SpatialPoint point : points) {
            graph.addVertex(point);
        }

        // Connect nearby points
        for (SpatialPoint from : points) {
            for (SpatialPoint to : points) {
                if (from.equals(to)) {
                    continue; // Skip self
                }

                double distance = GeodeticCalculator.vincentyDistance(
                        from.getCoordinate(), to.getCoordinate());

                if (distance <= maxConnectDistanceNM) {
                    TrajectorySegment edge = edgeFactory.createEdge(from, to);
                    graph.addEdge(from, to, edge);
                    graph.setEdgeWeight(edge, costFunction.getCost(edge));

                    // If undirected, no need to add reverse edge as SimpleWeightedGraph is undirected
                    if (!directedGraph) {
                        TrajectorySegment reverseEdge = edgeFactory.createEdge(to, from);
                        graph.addEdge(to, from, reverseEdge);
                        graph.setEdgeWeight(reverseEdge, costFunction.getCost(reverseEdge));
                    }
                }
            }
        }

        return graph;
    }

    /**
     * Create a cost function that considers obstacles
     *
     * @param baseCostFunction Base cost function for normal edges
     * @param obstacles Set of obstacles to avoid
     * @param obstaclePenaltyFactor Penalty factor for edges that come close to obstacles
     * @param obstacleClearanceNM Distance in nautical miles where penalty starts to apply
     * @return A cost function that penalizes edges near obstacles
     */
    public static CostFunction<SpatialPoint, TrajectorySegment> createObstacleAwareCostFunction(
            CostFunction<SpatialPoint, TrajectorySegment> baseCostFunction,
            Set<? extends SpatialElement> obstacles,
            double obstaclePenaltyFactor,
            double obstacleClearanceNM) {

        return new CostFunction<SpatialPoint, TrajectorySegment>() {
            @Override
            public double getCost(TrajectorySegment edge) {
                double baseCost = baseCostFunction.getCost(edge);

                // Check distance to obstacles
                double minDistance = Double.POSITIVE_INFINITY;

                for (SpatialElement obstacle : obstacles) {
                    // Calculate minimum distance from edge to obstacle
                    if (edge.intersects(obstacle)) {
                        return Double.POSITIVE_INFINITY; // Edge intersects obstacle, infinite cost
                    }

                    double distance = Double.POSITIVE_INFINITY;

                    if (obstacle instanceof SpatialPoint) {
                        SpatialPoint obstaclePoint = (SpatialPoint) obstacle;
                        distance = GeodeticCalculator.vincentyDistance(
                                edge.getPointAtFraction(0.5).getCoordinate(), obstaclePoint.getCoordinate());
                    } else if (obstacle instanceof SpatialLine) {
                        SpatialLine obstacleLine = (SpatialLine) obstacle;
                        distance = GeodeticCalculator.minimumDistance(
                                edge.toSpatialLine(), obstacleLine);
                    } else if (obstacle instanceof SpatialVolume) {
                        SpatialVolume obstacleVolume = (SpatialVolume) obstacle;
                        distance = GeodeticCalculator.minimumDistance(
                                edge.toSpatialLine(), obstacleVolume);
                    } else {
                        // Handle other obstacle types if necessary
                        distance = 0; // Default to 0 for now
                    }

                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                }

                // Apply penalty for edges that come close to obstacles
                double penalty = 1.0;
                if (minDistance < obstacleClearanceNM) {
                    // Penalty increases as distance decreases, up to the penalty factor
                    penalty = 1.0 + (obstacleClearanceNM - minDistance) /
                            obstacleClearanceNM * obstaclePenaltyFactor;
                }

                return baseCost * penalty;
            }

            @Override
            public double getCost(SpatialPoint from, SpatialPoint to) {
                return baseCostFunction.getCost(from, to);
            }
        };
    }

    /**
     * Create a heuristic function that considers obstacles
     * This is a more sophisticated heuristic that can help find better paths around obstacles
     *
     * @param baseHeuristic Base heuristic function
     * @param obstacles Set of obstacles to consider
     * @param obstacleClearanceNM Clearance distance in nautical miles
     * @return A heuristic function that accounts for obstacles
     */
    public static HeuristicFunction<SpatialPoint> createObstacleAwareHeuristic(
            HeuristicFunction<SpatialPoint> baseHeuristic,
            Set<? extends SpatialElement> obstacles,
            double obstacleClearanceNM) {

        return (current, goal) -> {
            double baseEstimate = baseHeuristic.estimate(current, goal);

            double distance;
            distance = GeodeticCalculator.vincentyDistance(current.getCoordinate(), goal.getCoordinate());

            // Check if direct path to goal intersects any obstacles
            SpatialLine directPath = SpatialLine.builder()
                    .id("direct-path")
                    .startPoint(current)
                    .endPoint(goal)
                    .build();
            double minDistance = Double.POSITIVE_INFINITY;
            for (SpatialElement obstacle : obstacles) {
                if (directPath.intersects(obstacle)) {
                    return Double.POSITIVE_INFINITY; // Direct path intersects obstacle, infinite cost
                }

                // Calculate minimum distance from direct path to obstacle
                // This is a simplified approach, in a real system you would use
                // more accurate calculations based on the specific obstacle type
                // For simplicity, use a point on the direct path (middle)
                SpatialPoint midPoint = directPath.getPointAtFraction(0.5);

                // Find closest point on the obstacle to the midpoint
                // This is a simplification - in a real system you would calculate
                // the true minimum distance from the direct path to the obstacle
                distance = Double.POSITIVE_INFINITY;

                if (obstacle instanceof SpatialPoint) {
                    SpatialPoint obstaclePoint = (SpatialPoint) obstacle;
                    distance = GeodeticCalculator.vincentyDistance(
                            midPoint.getCoordinate(), obstaclePoint.getCoordinate());
                } else {
                    // For other obstacle types, use the bounding box closest point as approximation
                    // This is another simplification
                    // In a real system, you would have specific distance calculations for each type
                    distance = 0; // Default to 0 for now
                }

                if (distance < minDistance) {
                    minDistance = distance;
                }

                // Apply penalty for direct path that comes close to obstacles
                double penalty = 1.0;
                if (minDistance < obstacleClearanceNM) {
                    // Penalty increases as distance decreases, up to the penalty factor
                    penalty = 1.0 + (obstacleClearanceNM - minDistance) /
                            obstacleClearanceNM * 10; // Increase penalty by 10%
                }

                return baseEstimate * penalty;
            }

            return baseEstimate;
        };
    }
}