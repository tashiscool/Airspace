package org.tash;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.function.Consumer;

import edu.uci.ics.jung.graph.DirectedSparseMultigraph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.tash.core.SpatialElement;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import org.tash.data.Vector3D;
import org.tash.event.AirspaceEvent;
import org.tash.event.AirspaceEventManager;
import org.tash.event.AirspaceInfringementEvent;
import org.tash.event.conflict.*;
import org.tash.flight.FlightTrajectory;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;
import org.tash.spatial.index.QuadtreeSpatialIndex;
import org.tash.spatial.index.SpatialIndex;
import org.tash.time.MotionModel;
import org.tash.trajectory.*;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultDirectedGraph;

/**
 * Highly polymorphic model for airspace management
 */
public class AirspaceModel {

    // Constants for minimum separation
    public static final double MIN_HORIZONTAL_SEPARATION_NM = 5.0; // 5 nautical miles
    public static final double MIN_VERTICAL_SEPARATION_FT = 1000.0; // 1000 feet
    public static final int MIN_TIME_SEPARATION_SEC = 60; // 1 minute

    // Graph structure for the airspace network
    private DirectedSparseMultigraph<SpatialPoint, TrajectorySegment> airspaceGraph;

    // Trajectories in the airspace
    private Map<String, FlightTrajectory> flightTrajectories;

    // Reserved airspace volumes
    private List<SpatialVolume> reservedVolumes;

    // Spatial index for fast spatial queries
    private SpatialIndex spatialIndex;

    // Event manager for notification
    private AirspaceEventManager eventManager;

    // Conflict detection strategies
    private Map<String, ConflictDetectionStrategy> conflictStrategies;

    /**
     * Constructor
     */
    public AirspaceModel() {
        this.airspaceGraph = new DirectedSparseMultigraph<>();
        this.flightTrajectories = new HashMap<>();
        this.reservedVolumes = new ArrayList<>();
        this.spatialIndex = new QuadtreeSpatialIndex();
        this.eventManager = new AirspaceEventManager();
        this.conflictStrategies = new HashMap<>();

        // Register default conflict detection strategies
        registerConflictStrategy("standard", new StandardConflictDetection());
        registerConflictStrategy("emergency", new EmergencyConflictDetection());
    }

    /**
     * Register a conflict detection strategy
     */
    public void registerConflictStrategy(String name, ConflictDetectionStrategy strategy) {
        conflictStrategies.put(name, strategy);
    }
    public ConflictDetectionStrategy getConflictStrategy(String name) {
        return conflictStrategies.get(name);
    }
    public List<ConflictDetectionStrategy> getConflictStrategies() {
        return new ArrayList<>(conflictStrategies.values());
    }

    /**
     * Calculate the closest point of approach between two trajectory segments
     */
    public ClosestPointOfApproach calculateClosestPointOfApproach(
            TrajectorySegment segment1, TrajectorySegment segment2) {
        // Get motion models
        MotionModel model1 = segment1.getMotionModel();
        MotionModel model2 = segment2.getMotionModel();

        // Get time overlap
        ZonedDateTime startTime = segment1.getStartTime().isBefore(segment2.getStartTime()) ?
                segment2.getStartTime() : segment1.getStartTime();
        ZonedDateTime endTime = segment1.getEndTime().isAfter(segment2.getEndTime()) ?
                segment2.getEndTime() : segment1.getEndTime();

        // Sample points in time to find closest approach
        int numSamples = 20;
        double minDistance = Double.MAX_VALUE;
        ZonedDateTime cpaTime = startTime;
        double cpaHorizontalDistance = 0;
        double cpaVerticalDistance = 0;

        for (int i = 0; i <= numSamples; i++) {
            double fraction = (double) i / numSamples;
            long durationMillis = java.time.Duration.between(startTime, endTime).toMillis();
            long offsetMillis = (long) (durationMillis * fraction);
            ZonedDateTime sampleTime = startTime.plus(java.time.Duration.ofMillis(offsetMillis));

            // Get positions at this time
            GeoCoordinate pos1 = model1.getPositionAt(sampleTime);
            GeoCoordinate pos2 = model2.getPositionAt(sampleTime);

            // Calculate distances
            double horizontalDistance = pos1.distanceTo(pos2);
            double verticalDistance = pos1.getAltitude() - pos2.getAltitude();

            // Check if this is the closest approach so far
            if (horizontalDistance < minDistance) {
                minDistance = horizontalDistance;
                cpaTime = sampleTime;
                cpaHorizontalDistance = horizontalDistance;
                cpaVerticalDistance = verticalDistance;
            }
        }

        return ClosestPointOfApproach.builder()
                .time(cpaTime)
                .horizontalDistance(cpaHorizontalDistance)
                .verticalDistance(cpaVerticalDistance)
                .build();
    }
    /**
     * Add a waypoint to the model
     */
    public SpatialPoint addWaypoint(double latitude, double longitude, double altitude, String id) {
        SpatialPoint waypoint = SpatialPoint.builder()
                .id(id != null ? id : "WP-" + UUID.randomUUID().toString())
                .coordinate(GeoCoordinate.builder()
                        .latitude(latitude)
                        .longitude(longitude)
                        .altitude(altitude)
                        .build())
                .build();

        spatialIndex.add(waypoint);
        airspaceGraph.addVertex(waypoint);

        return waypoint;
    }

    /**
     * Add a linear trajectory segment to the model
     */
    public LinearTrajectorySegment addLinearSegment(SpatialPoint source, SpatialPoint target,
                                                    ZonedDateTime startTime, ZonedDateTime endTime,
                                                    TrajectoryType type, String id) {
        LinearTrajectorySegment segment = LinearTrajectorySegment.builder()
                .id(id != null ? id : "SEG-" + UUID.randomUUID().toString())
                .source(source)
                .target(target)
                .startTime(startTime)
                .endTime(endTime)
                .type(type)
                .build();
        airspaceGraph.addEdge(segment, source, target);
        return segment;
    }

    /**
     * Get a point at a specific fraction along the line
     */
    public SpatialPoint getPointAtFraction(double fraction) {
        // Clamp fraction between 0 and 1
        fraction = Math.max(0, Math.min(1, fraction));

        // Interpolate between start and end
        SpatialPoint startPoint = new SpatialPoint();
        SpatialPoint endPoint = new SpatialPoint();
        if (fraction == 0) {
            return startPoint;
        }
        if (fraction == 1) {
            return endPoint;
        }
        // Interpolate between coordinates
        GeoCoordinate start = startPoint.getCoordinate();
        GeoCoordinate end = endPoint.getCoordinate();
        GeoCoordinate interpolated = start.interpolate(end, fraction);

        String id = startPoint.getId() + "-" + endPoint.getId();
        return SpatialPoint.builder()
                .id(id + "-point-" + fraction)
                .coordinate(interpolated)
                .build();
    }

    /**
     * Check if this line contains a point (approximately)
     */
    public boolean containsPoint(GeoCoordinate point) {

        SpatialPoint startPoint = new SpatialPoint();
        SpatialPoint endPoint = new SpatialPoint();
        Vector3D v1 = Vector3D.fromGeoCoordinate(startPoint.getCoordinate());
        Vector3D v2 = Vector3D.fromGeoCoordinate(endPoint.getCoordinate());
        Vector3D vp = Vector3D.fromGeoCoordinate(point);

        // Calculate squared distances
        double lineLengthSquared = v2.subtract(v1).magnitudeSquared();
        if (lineLengthSquared < 1e-10) {
            // Line is actually a point
            return v1.subtract(vp).magnitude() < 0.1; // Within 0.1 NM
        }

        // Calculate closest point on line to point
        double t = vp.subtract(v1).dotProduct(v2.subtract(v1)) / lineLengthSquared;
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        Vector3D closestPoint = v1.add(v2.subtract(v1).multiply(t));
        double distance = closestPoint.subtract(vp).magnitude();

        // Consider the point on the line if it's within a small threshold
        return distance < 0.1; // Within 0.1 NM
    }

    public void addEventListener(String eventType, Consumer<AirspaceEvent> listener) {
        eventManager.addEventListener(eventType, listener);
    }

    public CurvedTrajectorySegment addCurvedSegment(SpatialPoint wp2,
                                                    SpatialPoint wp4,
                                                    SpatialPoint cp1,
                                                    ZonedDateTime zonedDateTime,
                                                    ZonedDateTime zonedDateTime1,
                                                    TrajectoryType trajectoryType,
                                                    String s) {
        ZonedDateTime startTime = zonedDateTime;
        ZonedDateTime endTime = zonedDateTime1;
        String id = s;
        CurvedTrajectorySegment segment = CurvedTrajectorySegment.builder()
                .id(id != null ? id : "CURVED-SEG-" + UUID.randomUUID().toString())
                .source(wp2)
                .target(wp4)
                .controlPoint(cp1)
                .startTime(startTime)
                .endTime(endTime)
                .type(trajectoryType)
                .build();
        airspaceGraph.addEdge(segment, wp2, wp4);
        return segment;
    }

    public FlightTrajectory addFlightTrajectory(String lh456, String s) {
        String id = s;
        FlightTrajectory trajectory = FlightTrajectory.builder()
                .id(id != null ? id : "TRAJ-" + UUID.randomUUID().toString())
                .callsign(lh456)
                .build();
        flightTrajectories.put(trajectory.getId(), trajectory);
        return trajectory;
    }

    public HoldingPatternTrajectorySegment addHoldingPattern(SpatialPoint holdEntry, SpatialPoint holdExit, SpatialPoint holdCenter, double v, int i, int i1, ZonedDateTime zonedDateTime, ZonedDateTime zonedDateTime1, String hold1) {
        ZonedDateTime startTime = zonedDateTime;
        ZonedDateTime endTime = zonedDateTime1;
        String id = hold1;
        HoldingPatternTrajectorySegment segment = HoldingPatternTrajectorySegment.builder()
                .id(id != null ? id : "HOLD-SEG-" + UUID.randomUUID().toString())
                        .source(holdEntry)
                        .target(holdExit)
                        .centerPoint(holdCenter)
                        .startTime(startTime)
                        .endTime(endTime)
                        .build();
        airspaceGraph.addEdge(segment, holdEntry, holdExit);
        return segment;
    }

    public SpatialVolume reserveAirspace(List<SpatialPoint> boundaryPoints, int i, int i1, ZonedDateTime zonedDateTime, ZonedDateTime zonedDateTime1, String reserved1) {
        ZonedDateTime startTime = zonedDateTime;
        ZonedDateTime endTime = zonedDateTime1;
        String id = reserved1;
        SpatialVolume volume = SpatialVolume.builder()
                .id(id != null ? id : "VOL-" + UUID.randomUUID().toString())
                .startTime(startTime)
                .endTime(endTime)
                .basePolygon(toBasePolygon(boundaryPoints))
                .build();
        reservedVolumes.add(volume);
        return volume;
    }

    private SpatialPolygon toBasePolygon(List<SpatialPoint> boundaryPoints) {
        return new SpatialPolygon(boundaryPoints);
    }

    public List<SpatialElement> findElementsWithin(BoundingBox queryBox) {
        return spatialIndex.query(queryBox);
    }

    public List<SeparationConflict> checkTrajectoryConflicts(String strategyName) {
        List<SeparationConflict> conflicts = new ArrayList<>();

        // Check each pair of segments
        for (SpatialPoint segment1 : airspaceGraph.getVertices()) {
            for (SpatialPoint segment2 : airspaceGraph.getVertices()) {
                if (segment1 == segment2) {
                    continue;
                }

                if (airspaceGraph.isNeighbor(segment1, segment2)) {
                    continue;
                }
                if ( segment1 instanceof TrajectorySegment && segment2 instanceof TrajectorySegment) {
                    SeparationConflict conflict = detectConflict((TrajectorySegment) segment1, (TrajectorySegment) segment2);
                    if (conflict != null) {
                        conflicts.add(conflict);
                    }
                }
            }
        }

        return conflicts;
    }

    private SeparationConflict detectConflict(TrajectorySegment segment1, TrajectorySegment segment2) {
        // First check temporal overlap
        if (!segment1.timeOverlaps(segment2)) {
            return null;
        }

        // Calculate closest point of approach
        ClosestPointOfApproach cpa = calculateClosestPointOfApproach(segment1, segment2);

        // Check if separation standards are violated
        if (cpa.getHorizontalDistance() < MIN_HORIZONTAL_SEPARATION_NM) {
            if (Math.abs(cpa.getVerticalDistance()) < MIN_VERTICAL_SEPARATION_FT) {
                // Create a conflict
                return SeparationConflict.builder()
                        .segment1(segment1)
                        .segment2(segment2)
                        .time(cpa.getTime())
                        .horizontalSeparation(cpa.getHorizontalDistance())
                        .verticalSeparation(cpa.getVerticalDistance())
                        .build();
            }
        }

        return null;
    }

    public List<AirspaceInfringementEvent> checkAirspaceInfringements() {
        List<AirspaceInfringementEvent> infringements = new ArrayList<>();

        for (FlightTrajectory trajectory : flightTrajectories.values()) {
            for (SpatialVolume volume : reservedVolumes) {
                if (trajectory.intersectsVolume(volume)) {
                    trajectory.getAllSegments().forEach(
                            segment -> infringements.add(AirspaceInfringementEvent.builder().segment(segment).volume(volume).build())
                    );
                }
            }
        }

        return infringements;
    }

    public List<TrajectorySegment> findShortestPath(SpatialPoint wp4, SpatialPoint wp7) {
        DefaultDirectedGraph<SpatialPoint, TrajectorySegment> graph = new DefaultDirectedGraph<>(TrajectorySegment.class);
        for (SpatialPoint vertex : airspaceGraph.getVertices()) {
            graph.addVertex(vertex);
        }
        for (TrajectorySegment edge : airspaceGraph.getEdges()) {
            graph.addEdge(airspaceGraph.getSource(edge), airspaceGraph.getDest(edge), edge);
        }

        // Use Dijkstra's algorithm to find the shortest path
        DijkstraShortestPath<SpatialPoint, TrajectorySegment> dijkstraAlg = new DijkstraShortestPath<>(graph);
        GraphPath<SpatialPoint, TrajectorySegment> path = dijkstraAlg.getPath(wp4, wp7);

        // Convert the path to a list of TrajectorySegments
        if (path != null) {
            return path.getEdgeList();
        } else {
            return new ArrayList<>();
        }
    }
}
