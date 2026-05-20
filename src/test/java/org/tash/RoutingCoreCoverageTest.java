package org.tash;

import org.jgrapht.Graph;
import org.junit.jupiter.api.Test;
import org.tash.core.routing.lib.AStarPathPlanner;
import org.tash.core.routing.lib.CostFunction;
import org.tash.core.routing.lib.EdgeFactory;
import org.tash.core.routing.raw.AltitudeConstraint;
import org.tash.core.routing.raw.ClimbDescentRateConstraint;
import org.tash.core.routing.raw.MinimumDistanceConstraint;
import org.tash.core.routing.raw.RestrictedAirspaceConstraint;
import org.tash.core.routing.raw.RoutingNode;
import org.tash.core.routing.raw.RoutingResult;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectorySegment;
import org.tash.trajectory.TrajectoryType;

import java.time.ZonedDateTime;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

class RoutingCoreCoverageTest {
    private final ZonedDateTime time = ZonedDateTime.parse("2026-05-20T12:00:00Z");

    @Test
    void rawRoutingConstraintsAndResultsEvaluateOperationalPathRules() {
        SpatialPoint low = point("low", 0, 0, 5000);
        SpatialPoint cruise = point("cruise", 0, 1, 25000);
        SpatialPoint high = point("high", 0, 2, 45000);

        AltitudeConstraint altitude = new AltitudeConstraint(10000, 40000, 25, true);
        assertTrue(altitude.isViolated(low, cruise));
        assertFalse(altitude.isViolated(cruise, point("cruise2", 0, 1.5, 26000)));
        assertEquals(25, altitude.getPenalty(), 0.0001);
        assertTrue(altitude.isHardConstraint());

        ClimbDescentRateConstraint climb = new ClimbDescentRateConstraint(100, 100, 5, false);
        assertTrue(climb.isViolated(cruise, high));
        assertFalse(climb.isViolated(cruise, cruise));

        MinimumDistanceConstraint distance = new MinimumDistanceConstraint(point("ref", 0, 0.5, 25000), 25, 10, true);
        assertTrue(distance.isViolated(cruise, point("near", 0, 0.8, 25000)));
        assertSame(distance.getSpatialElement(), distance.getSpatialElement());

        RestrictedAirspaceConstraint restricted = new RestrictedAirspaceConstraint(
                SpatialLine.builder()
                        .id("restricted-line")
                        .startPoint(point("r-a", -0.5, 0.5, 25000))
                        .endPoint(point("r-b", 0.5, 0.5, 25000))
                        .build(), 100, true);
        assertTrue(restricted.isViolated(point("route-a", 0, 0, 25000), point("route-b", 0, 1, 25000)));
        assertFalse(restricted.isViolated(point("far-a", 2, 0, 25000), point("far-b", 2, 1, 25000)));

        RoutingNode parent = new RoutingNode(cruise, null, 3, 4);
        RoutingNode child = new RoutingNode(high, parent, 2, 2);
        assertEquals(7, parent.getFCost(), 0.0001);
        assertSame(parent, child.getParent());
        assertTrue(child.compareTo(parent) < 0);
        assertEquals(parent, new RoutingNode(cruise, null, 100, 100));

        RoutingResult result = new RoutingResult(List.of(cruise, high), 100, 110, Set.of(restricted));
        assertTrue(result.isSuccessful());
        assertEquals(2, result.getWaypoints().size());
        assertEquals(1, result.getViolatedConstraints().size());
        assertFalse(new RoutingResult(List.of(), 0, 0, Set.of()).isSuccessful());
    }

    @Test
    void aStarPathPlannerFindsConstrainedAndAlternativeRoutes() {
        SpatialPoint a = point("a", 0, 0, 20000);
        SpatialPoint b = point("b", 0, 0.5, 20000);
        SpatialPoint c = point("c", 0.5, 0.5, 20000);
        SpatialPoint d = point("d", 0.5, 1.0, 20000);
        SpatialPoint e = point("e", 0, 1.0, 20000);
        Set<SpatialPoint> points = new HashSet<>(List.of(a, b, c, d, e));
        EdgeFactory<SpatialPoint, TrajectorySegment> edgeFactory = this::edge;
        CostFunction<SpatialPoint, TrajectorySegment> cost = AStarPathPlanner.createDefaultCostFunction();
        Graph<SpatialPoint, TrajectorySegment> graph = AStarPathPlanner.createGraphFromPoints(
                points, 45.0, true, edgeFactory, cost);

        AStarPathPlanner planner = new AStarPathPlanner(graph, time, 480, cost,
                AStarPathPlanner.createDefaultHeuristicFunction(), edgeFactory);
        List<TrajectorySegment> direct = planner.findPath(a, e);
        assertFalse(direct.isEmpty());

        SpatialPolygon obstacle = square("block", -0.1, 0.4, 0.1, 0.6, 20000);
        List<TrajectorySegment> obstacleAvoiding = planner.findPath(a, e, Set.of(obstacle));
        assertFalse(obstacleAvoiding.isEmpty());
        assertTrue(obstacleAvoiding.stream().noneMatch(segment -> segment.intersects(obstacle)));

        List<TrajectorySegment> constrained = planner.findPathWithConstraints(a, e,
                segment -> !segment.getTarget().equals(b));
        assertFalse(constrained.isEmpty());
        assertTrue(constrained.stream().noneMatch(segment -> segment.getTarget().equals(b)));

        assertFalse(planner.findMultiplePaths(a, e, 2).isEmpty());
        assertTrue(planner.findMultiplePaths(a, e, 0).isEmpty());
        assertFalse(planner.findMultiplePathsWithConstraints(a, e, segment -> true, 2).isEmpty());

        double baseCost = cost.getCost(direct.get(0));
        CostFunction<SpatialPoint, TrajectorySegment> obstacleCost =
                AStarPathPlanner.createObstacleAwareCostFunction(cost, Set.of(obstacle), 10, 20);
        assertTrue(obstacleCost.getCost(direct.get(0)) >= baseCost);
        assertTrue(AStarPathPlanner.createObstacleAwareHeuristic(
                AStarPathPlanner.createDefaultHeuristicFunction(), Set.of(obstacle), 20).estimate(a, e) >= 0);
    }

    private TrajectorySegment edge(SpatialPoint from, SpatialPoint to) {
        return LinearTrajectorySegment.builder()
                .id(from.getId() + "-" + to.getId())
                .source(from)
                .target(to)
                .startTime(time)
                .endTime(time.plusMinutes(30))
                .type(TrajectoryType.DIRECT)
                .build();
    }

    private SpatialPolygon square(String id, double minLat, double minLon, double maxLat, double maxLon, double altitude) {
        return SpatialPolygon.builder().id(id).vertices(List.of(
                point(id + "-a", minLat, minLon, altitude),
                point(id + "-b", minLat, maxLon, altitude),
                point(id + "-c", maxLat, maxLon, altitude),
                point(id + "-d", maxLat, minLon, altitude))).build();
    }

    private SpatialPoint point(String id, double lat, double lon, double alt) {
        return SpatialPoint.builder()
                .id(id)
                .coordinate(GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(alt).build())
                .build();
    }
}
