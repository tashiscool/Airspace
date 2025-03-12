package org.tash.core.routing.replan;

import org.tash.core.routing.raw.RoutingConstraint;
import org.tash.core.routing.search.astar.*;
import org.tash.core.routing.search.rrt.RRTConfig;
import org.tash.core.routing.search.rrt.RRTRoutePlanner;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.trajectory.TrajectorySegment;

import java.util.*;

/**
 * Demonstration of the route planning algorithms
 */
public class RoutePlanningDemo {
    
    public static void main(String[] args) {
        // Demo for route planning capabilities
        astarPlanningDemo();
        rrtPlanningDemo();
        dynamicReroutingDemo();
    }
    
    /**
     * Demonstrate A* pathfinding with custom heuristics
     */
    private static void astarPlanningDemo() {
        System.out.println("===== A* Route Planning Demo =====");
        
        // Create waypoints
        SpatialPoint start = SpatialPoint.builder()
            .id("START")
            .coordinate(GeoCoordinate.builder()
                .latitude(37.6213)
                .longitude(-122.3790)
                .altitude(5000.0)
                .build())
            .build();
            
        SpatialPoint end = SpatialPoint.builder()
            .id("END")
            .coordinate(GeoCoordinate.builder()
                .latitude(40.6413)
                .longitude(-73.7781)
                .altitude(5000.0)
                .build())
            .build();

        List<GeoCoordinate> l1 = Arrays.asList(
                GeoCoordinate.builder()
                        .latitude(37.6157)
                        .longitude(-122.3719)
                        .build(),
                GeoCoordinate.builder()
                        .latitude(37.6157)
                        .longitude(-122.3790)
                        .build(),
                GeoCoordinate.builder()
                        .latitude(37.6319)
                        .longitude(-122.3790)
                        .build(),
                GeoCoordinate.builder()
                        .latitude(37.6319)
                        .longitude(-122.3719)
                        .build()
        );

        List<GeoCoordinate> l2 = Arrays.asList(
                GeoCoordinate.builder()
                        .latitude(40.6319)
                        .longitude(-73.7781)
                        .build(),
                GeoCoordinate.builder()
                        .latitude(40.6319)
                        .longitude(-73.7710)
                        .build(),
                GeoCoordinate.builder()
                        .latitude(40.6157)
                        .longitude(-73.7710)
                        .build(),
                GeoCoordinate.builder()
                        .latitude(40.6157)
                        .longitude(-73.7781)
                        .build()
        );
        
        // Create some constraints
        List<SpatialPoint> l2o = new ArrayList<>();
        List<SpatialPoint> l1o  = new ArrayList<>();
        Set<RoutingConstraint> constraints = createSampleConstraints(
                start, end,
                SpatialPolygon.builder()
                   .id("AREA1")
                   .vertices(l1o)
                   .build(),
                SpatialPolygon.builder()
                   .id("AREA2")
                   .vertices(l2o)
                     .build()
        );
        
        // Create different heuristic functions
        System.out.println("\n--- Testing Different Heuristics ---");
        
        // Standard A* with geodetic heuristic
        AStarConfig configStandard = AStarConfig.builder()
            .nodeSpacingNM(50.0)
            .altitudeStepFt(2000.0)
            .maxIterations(1000)
            .pathSmoothingEnabled(true)
            .build();
            
        AStarRoutePlanner plannerStandard = new AStarRoutePlanner(
            new DefaultNodeGenerator(),
            new GeodeticHeuristic(),
            configStandard
        );
        
        System.out.println("Standard A* Planning with Geodetic Heuristic:");
        List<SpatialPoint> routeStandard = plannerStandard.findRoute(start, end, constraints);
        printRouteInfo(routeStandard);
        
        // A* with altitude-aware heuristic
        AStarConfig configAltitude = AStarConfig.builder()
            .nodeSpacingNM(50.0)
            .altitudeStepFt(2000.0)
            .maxIterations(1000)
            .pathSmoothingEnabled(true)
            .build();
            
        AStarRoutePlanner plannerAltitude = new AStarRoutePlanner(
            new DefaultNodeGenerator(),
            new AltitudeAwareHeuristic(10.0),
            configAltitude
        );
        
        System.out.println("\nA* Planning with Altitude-Aware Heuristic:");
        List<SpatialPoint> routeAltitude = plannerAltitude.findRoute(start, end, constraints);
        printRouteInfo(routeAltitude);
        
        // A* with constraint-aware heuristic
        AStarConfig configConstraint = AStarConfig.builder()
            .nodeSpacingNM(50.0)
            .altitudeStepFt(2000.0)
            .maxIterations(1000)
            .pathSmoothingEnabled(true)
            .build();
            
        AStarRoutePlanner plannerConstraint = new AStarRoutePlanner(
            new DefaultNodeGenerator(),
            new ConstraintAwareHeuristic(2.0),
            configConstraint
        );
        
        System.out.println("\nA* Planning with Constraint-Aware Heuristic:");
        List<SpatialPoint> routeConstraint = plannerConstraint.findRoute(start, end, constraints);
        printRouteInfo(routeConstraint);
        
        // A* with adaptive expansion strategy
        AStarConfig configAdaptive = AStarConfig.builder()
            .nodeSpacingNM(50.0)
            .altitudeStepFt(2000.0)
            .maxIterations(1000)
            .pathSmoothingEnabled(true)
            .expansionStrategy(new AdaptiveExpansionStrategy())
            .build();
            
        AStarRoutePlanner plannerAdaptive = new AStarRoutePlanner(
            new DefaultNodeGenerator(),
            new GeodeticHeuristic(),
            configAdaptive
        );
        
        System.out.println("\nA* Planning with Adaptive Expansion Strategy:");
        List<SpatialPoint> routeAdaptive = plannerAdaptive.findRoute(start, end, constraints);
        printRouteInfo(routeAdaptive);
        
        // Convert to trajectory segments
        List<TrajectorySegment> trajectorySegments = plannerAdaptive.waypointsToTrajectory(routeAdaptive);
        System.out.println("\nTrajectory segments created: " + trajectorySegments.size());
    }

    private static Set<RoutingConstraint> createSampleConstraints(SpatialPoint start, SpatialPoint end, SpatialPolygon area1, SpatialPolygon area2)
    {
        Set<RoutingConstraint> constraints = new HashSet<>();

//        // Example: Max speed constraint
//        constraints.add(new MaxSpeedConstraint(10.0));
//
//        // Example: Max acceleration constraint
//        constraints.add(new MaxAccelerationConstraint(1.0));
//
//        // Example: Max deceleration constraint
//        constraints.add(new MaxDecelerationConstraint(1.0));
//
//        // Example: Max distance constraint
//        constraints.add(new MaxDistanceConstraint(500.0));
//
//        // Example: Altitude constraint
//        constraints.add(new AltitudeConstraint(4000.0, 6000.0));
//
//        // Example: Area constraint
//        constraints.add(new AreaConstraint(area1));
//        constraints.add(new AreaConstraint(area2));
//
//        // Example: Avoidance constraint
//        constraints.add(new AvoidanceConstraint(start, end, 10.0));

        return constraints;
    }

    private static Set<RoutingConstraint> createSampleConstraints() {
        Set<RoutingConstraint> constraints = new HashSet<>();

//        // Example: Max speed constraint
//        constraints.add(new MaxSpeedConstraint(10.0));
//
//        // Example: Max acceleration constraint
//        constraints.add(new MaxAccelerationConstraint(1.0));
//
//        // Example: Max deceleration constraint
//        constraints.add(new MaxDecelerationConstraint(1.0));
//
//        // Example: Max distance constraint
//        constraints.add(new MaxDistanceConstraint(500.0));

        return constraints;
    }

    /**
     * Demonstrate RRT pathfinding
     */
    private static void rrtPlanningDemo() {
        System.out.println("\n===== RRT Route Planning Demo =====");
        
        // Create waypoints
        SpatialPoint start = SpatialPoint.builder()
            .id("START")
            .coordinate(GeoCoordinate.builder()
                .latitude(37.6213)
                .longitude(-122.3790)
                .altitude(5000.0)
                .build())
            .build();
            
        SpatialPoint end = SpatialPoint.builder()
            .id("END")
            .coordinate(GeoCoordinate.builder()
                .latitude(40.6413)
                .longitude(-73.7781)
                .altitude(5000.0)
                .build())
            .build();
        
        // Create some constraints
        Set<RoutingConstraint> constraints = createSampleConstraints();
        
        // Test different RRT configurations
        System.out.println("\n--- Testing Different RRT Configurations ---");
        
        // Standard RRT
        RRTConfig configStandard = RRTConfig.builder()
            .stepSizeNM(50.0)
            .maxIterations(1000)
            .goalBias(0.1)
            .useRRTStar(false)
            .pathSmoothingEnabled(true)
            .build();
            
        RRTRoutePlanner plannerStandard = new RRTRoutePlanner(configStandard);
        
        System.out.println("Standard RRT Planning:");
        List<SpatialPoint> routeStandard = plannerStandard.findRoute(start, end, constraints);
        printRouteInfo(routeStandard);
        
        // RRT* with path optimization
        RRTConfig configRRTStar = RRTConfig.builder()
            .stepSizeNM(50.0)
            .maxIterations(1000)
            .goalBias(0.1)
            .useRRTStar(true)
            .pathSmoothingEnabled(true)
            .build();
            
        RRTRoutePlanner plannerRRTStar = new RRTRoutePlanner(configRRTStar);
        
        System.out.println("\nRRT* Planning with Optimization:");
        List<SpatialPoint> routeRRTStar = plannerRRTStar.findRoute(start, end, constraints);
        printRouteInfo(routeRRTStar);
        
        // RRT with higher goal bias
        RRTConfig configHighBias = RRTConfig.builder()
            .stepSizeNM(50.0)
            .maxIterations(1000)
            .goalBias(0.3)
            .useRRTStar(true)
            .pathSmoothingEnabled(true)
            .build();
            
        RRTRoutePlanner plannerHighBias = new RRTRoutePlanner(configHighBias);
        
        System.out.println("\nRRT* Planning with Higher Goal Bias (0.3):");
        List<SpatialPoint> routeHighBias = plannerHighBias.findRoute(start, end, constraints);
        printRouteInfo(routeHighBias);
        
        // Convert to trajectory segments
        List<TrajectorySegment> trajectorySegments = plannerRRTStar.waypointsToTrajectory(routeRRTStar);
        System.out.println("\nTrajectory segments created: " + trajectorySegments.size());
    }
    
    /**
     * Demonstrate dynamic rerouting
     */
    private static void dynamicReroutingDemo() {
        System.out.println("\n===== Dynamic Rerouting Demo =====");
        
        // Create waypoints
        SpatialPoint start = SpatialPoint.builder()
            .id("START")
            .coordinate(GeoCoordinate.builder()
                .latitude(37.6213)
                .longitude(-122.3790)
                .altitude(5000.0)
                .build())
            .build();
            
        SpatialPoint end = SpatialPoint.builder()
            .id("END")
            .coordinate(GeoCoordinate.builder()
                .latitude(40.6413)
                .longitude(-73.7781)
                .altitude(5000.0)
                .build())
            .build();
        
        // Create some static constraints
        Set<RoutingConstraint> staticConstraints = createSampleConstraints();
        
        // Create a dynamic route planner
        AStarRoutePlanner basePlanner = new AStarRoutePlanner();
        DynamicRoutePlanner dynamicPlanner = new DynamicRoutePlanner(
            basePlanner,
            DynamicRoutePlanner.ReplanningSensorConfig.builder()
                .onRouteToleranceNM(5.0)
                .rerouteWhenOffTrack(true)
                .constraintPenaltyThreshold(10.0)
                .lookaheadTimeMinutes(15.0)
                .predictiveReplanning(true)
                .build()
        );
        
        // Add a listener for route change events
        dynamicPlanner.addRouteChangeListener(event -> {
            System.out.println("Route Change Event: " + event.getType());
            System.out.println("  Message: " + event.getMessage());
            System.out.println("  Timestamp: " + event.getTimestamp());
            
            if (event.getRoute().isValid()) {
                System.out.println("  New route has " + event.getRoute().getWaypoints().size() + " waypoints");
                System.out.println("  Total distance: " + String.format("%.1f", event.getRoute().getTotalDistance()) + " NM");
            }
        });
        
        // Plan initial route
        System.out.println("\n--- Initial Route Planning ---");
        DynamicRoutePlanner.DynamicRoute initialRoute = dynamicPlanner.planInitialRoute(start, end, staticConstraints);
        
        if (initialRoute.isValid()) {
            System.out.println("Initial route planned successfully");
            System.out.println("  Waypoints: " + initialRoute.getWaypoints().size());
            System.out.println("  Total distance: " + String.format("%.1f", initialRoute.getTotalDistance()) + " NM");
        } else {
            System.out.println("Failed to plan initial route");
            return;
        }
        List<SpatialPoint> routeInitial = initialRoute.getWaypoints();
        printRouteInfo(routeInitial);
        List<TrajectorySegment> trajectorySegmentsInitial = dynamicPlanner.waypointsToTrajectory(routeInitial);
        System.out.println("Initial trajectory segments: " + trajectorySegmentsInitial.size());
        System.out.println();
        System.out.println("--- Dynamic Rerouting ---");
        SpatialPoint offTrackPoint = SpatialPoint.builder()
            .id("OFF_TRACK")
            .coordinate(GeoCoordinate.builder()
                .latitude(38.0)
                .longitude(-120.0)
                .altitude(5000.0)
                .build())
            .build();
        DynamicRoutePlanner.DynamicRoute dynamicRoute = dynamicPlanner.dynamicReplan(routeInitial, offTrackPoint, staticConstraints);
        if (dynamicRoute.isValid()) {
            System.out.println("Dynamic reroute planned successfully");
            System.out.println("  Waypoints: " + dynamicRoute.getWaypoints().size());
            System.out.println("  Total distance: " + String.format("%.1f", dynamicRoute.getTotalDistance()) + " NM");
        } else {
            System.out.println("Failed to plan dynamic reroute");
            return;
        }
        List<SpatialPoint> routeReroute = dynamicRoute.getWaypoints();
        printRouteInfo(routeReroute);
        List<TrajectorySegment> trajectorySegmentsReroute = dynamicPlanner.waypointsToTrajectory(routeReroute);
        System.out.println("Reroute trajectory segments: " + trajectorySegmentsReroute.size());
    }

    private static void printRouteInfo(List<SpatialPoint> route) {
        System.out.println("Route waypoints: " + route.size());
        System.out.println("Total distance: " + String.format("%.1f", calculateTotalDistance(route)) + " NM");
    }
    private static double calculateTotalDistance(List<SpatialPoint> route) {
        double totalDistance = 0.0;
        for (int i = 0; i < route.size() - 1; i++) {
            totalDistance += route.get(i).distanceTo(route.get(i + 1));
        }
        return totalDistance;
    }
}