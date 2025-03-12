package org.tash;

import org.tash.core.*;
import org.tash.data.*;
import org.tash.event.*;
import org.tash.event.conflict.*;
import org.tash.flight.*;
import org.tash.spatial.*;
import org.tash.trajectory.*;
import org.tash.time.*;
import org.tash.visitor.*;
import org.tash.visitor.CountingVisitor;
import org.tash.visitor.FinderVisitor;

import java.time.ZonedDateTime;
import java.time.ZoneId;
import java.util.ArrayList;
import java.util.List;

/**
 * Demonstration of the highly polymorphic airspace model
 */
public class PolymorphicAirspaceModelDemo {

    public static void main(String[] args) {
        // Create the model
        AirspaceModel model = new AirspaceModel();
        
        // Register event listener for conflicts
        model.addEventListener("TRAJECTORY_CONFLICT", event -> {
            TrajectoryConflictEvent conflictEvent = (TrajectoryConflictEvent) event;
            System.out.println("CONFLICT DETECTED: ");
            System.out.println("  Segments: " + conflictEvent.getSegment1().getId() + 
                              " and " + conflictEvent.getSegment2().getId());
            System.out.println("  Time: " + conflictEvent.getTimestamp());
            System.out.println("  Horizontal separation: " + 
                              String.format("%.2f NM", conflictEvent.getHorizontalSeparation()));
            System.out.println("  Vertical separation: " + 
                              String.format("%.0f feet", conflictEvent.getVerticalSeparation()));
        });
        
        // Register event listener for airspace infringements
        model.addEventListener("AIRSPACE_INFRINGEMENT", event -> {
            AirspaceInfringementEvent infringementEvent = 
                (AirspaceInfringementEvent) event;
            System.out.println("AIRSPACE INFRINGEMENT: ");
            System.out.println("  Segment: " + infringementEvent.getSegment().getId());
            System.out.println("  Volume: " + infringementEvent.getVolume().getId());
            System.out.println("  Time: " + infringementEvent.getTimestamp());
        });
        
        // Create base time
        ZonedDateTime baseTime = ZonedDateTime.now(ZoneId.of("UTC"));
        
        // Create some waypoints
        SpatialPoint wp1 = model.addWaypoint(51.5, -0.1, 10000, "WP1");
        SpatialPoint wp2 = model.addWaypoint(51.6, 0.1, 15000, "WP2");
        SpatialPoint wp3 = model.addWaypoint(51.7, 0.3, 20000, "WP3");
        SpatialPoint wp4 = model.addWaypoint(51.5, 0.2, 12000, "WP4");
        SpatialPoint wp5 = model.addWaypoint(51.6, 0.3, 18000, "WP5");
        SpatialPoint wp6 = model.addWaypoint(51.8, 0.4, 22000, "WP6");
        SpatialPoint wp7 = model.addWaypoint(51.9, 0.5, 25000, "WP7");
        
        // Create a control point for curved segments
        SpatialPoint cp1 = model.addWaypoint(51.65, 0.2, 16500, "CP1");
        
        // Create a center point for holding patterns
        SpatialPoint holdCenter = model.addWaypoint(51.65, 0.25, 17000, "HOLD");
        SpatialPoint holdEntry = model.addWaypoint(51.62, 0.22, 17000, "HOLD_ENTRY");
        SpatialPoint holdExit2 = model.addWaypoint(51.67, 0.27, 17000, "HOLD_EXIT");
        
        // Create flight trajectory 1
        FlightTrajectory flight1 = model.addFlightTrajectory("BA123", "FLIGHT-BA123");
        
        // Add segments to flight 1
        LinearTrajectorySegment seg1 = model.addLinearSegment(
            wp1, wp2, baseTime, baseTime.plusMinutes(10), 
            TrajectoryType.STANDARD, "SEG1");
        flight1.addSegment(seg1);
        
        LinearTrajectorySegment seg2 = model.addLinearSegment(
            wp2, wp3, baseTime.plusMinutes(10), baseTime.plusMinutes(20), 
            TrajectoryType.STANDARD, "SEG2");
        flight1.addSegment(seg2);
        
        LinearTrajectorySegment seg3 = model.addLinearSegment(
            wp3, wp6, baseTime.plusMinutes(20), baseTime.plusMinutes(30), 
            TrajectoryType.STANDARD, "SEG3");
        flight1.addSegment(seg3);
        
        // Create an alternative path using a curved segment
        List<TrajectorySegment> altPath = new ArrayList<>();
        
        CurvedTrajectorySegment altSeg1 = model.addCurvedSegment(
            wp2, wp4, cp1, baseTime.plusMinutes(10), baseTime.plusMinutes(15), 
            TrajectoryType.CONTINGENCY, "ALT-SEG1");
        
        LinearTrajectorySegment altSeg2 = model.addLinearSegment(
            wp4, wp5, baseTime.plusMinutes(15), baseTime.plusMinutes(22), 
            TrajectoryType.CONTINGENCY, "ALT-SEG2");
            
        LinearTrajectorySegment altSeg3 = model.addLinearSegment(
            wp5, wp6, baseTime.plusMinutes(22), baseTime.plusMinutes(32), 
            TrajectoryType.CONTINGENCY, "ALT-SEG3");
            
        altPath.add(altSeg1);
        altPath.add(altSeg2);
        altPath.add(altSeg3);
        
        // Add the alternative path to flight 1
        flight1.addAlternativePath(wp2, altPath);
        
        // Define a join point
        flight1.joinAlternativePathsAt(wp6);
        
        // Create flight trajectory 2 with a holding pattern
        FlightTrajectory flight2 = model.addFlightTrajectory("LH456", "FLIGHT-LH456");
        
        // Add segments to flight 2
        LinearTrajectorySegment holdApproach = model.addLinearSegment(
            wp4, holdEntry, baseTime.plusMinutes(5), baseTime.plusMinutes(12), 
            TrajectoryType.STANDARD, "HOLD-APPROACH");
        flight2.addSegment(holdApproach);

        // Add holding pattern
        HoldingPatternTrajectorySegment holdingSegment = model.addHoldingPattern(
            holdEntry, holdExit2, holdCenter, 5.0, 90, 3,
            baseTime.plusMinutes(12), baseTime.plusMinutes(25), "HOLD1");
        flight2.addSegment(holdingSegment);
        
        // Add exit segment
        LinearTrajectorySegment holdExit = model.addLinearSegment(
            holdExit2, wp7, baseTime.plusMinutes(25), baseTime.plusMinutes(35),
            TrajectoryType.STANDARD, "HOLD-EXIT");
        flight2.addSegment(holdExit);
        
        // Create a reserved airspace volume
        List<SpatialPoint> boundaryPoints = new ArrayList<>();
        boundaryPoints.add(model.addWaypoint(51.65, 0.15, 15000, "B1"));
        boundaryPoints.add(model.addWaypoint(51.70, 0.20, 15000, "B2"));
        boundaryPoints.add(model.addWaypoint(51.68, 0.25, 15000, "B3"));
        boundaryPoints.add(model.addWaypoint(51.63, 0.22, 15000, "B4"));
        
        SpatialVolume reservedVolume = model.reserveAirspace(
            boundaryPoints, 15000, 25000, 
            baseTime.plusMinutes(15), baseTime.plusMinutes(45), "RESERVED1");
        
        // Demonstrate spatial indexing
        System.out.println("\nDemonstrating spatial indexing:");
        BoundingBox queryBox = BoundingBox.builder()
            .minLat(51.6)
            .maxLat(51.7)
            .minLon(0.2)
            .maxLon(0.3)
            .minAlt(16000)
            .maxAlt(20000)
            .startTime(baseTime.plusMinutes(10))
            .endTime(baseTime.plusMinutes(25))
            .build();
            
        List<SpatialElement> elementsInBox = model.findElementsWithin(queryBox);
        System.out.println("Found " + elementsInBox.size() + " elements within query box:");
        for (SpatialElement element : elementsInBox) {
            System.out.println("  " + element.getElementType() + ": " + element.getId());
        }
        
        // Demonstrate polymorphic intersection testing between different element types
        System.out.println("\nDemonstrating polymorphic intersection testing:");
        
        boolean curveSeg_holdingSeg = altSeg1.intersects(holdingSegment);
        System.out.println("Curved segment intersects holding pattern: " + curveSeg_holdingSeg);
        
        boolean linearSeg_volume = seg2.intersects(reservedVolume);
        System.out.println("Linear segment intersects reserved volume: " + linearSeg_volume);
        
        boolean holdingSeg_volume = holdingSegment.intersects(reservedVolume);
        System.out.println("Holding pattern intersects reserved volume: " + holdingSeg_volume);
        
        // Demonstrate conflict detection
        System.out.println("\nChecking for trajectory conflicts (standard rules):");
        List<SeparationConflict> conflicts = model.checkTrajectoryConflicts("standard");
        System.out.println("Found " + conflicts.size() + " conflicts");
        
        // Demonstrate conflict detection with different strategy
        System.out.println("\nChecking for trajectory conflicts (emergency rules):");
        conflicts = model.checkTrajectoryConflicts("emergency");
        System.out.println("Found " + conflicts.size() + " conflicts with emergency rules");
        
        // Demonstrate airspace infringement detection
        System.out.println("\nChecking for airspace infringements:");
        List<AirspaceInfringementEvent> infringements = model.checkAirspaceInfringements();
        System.out.println("Found " + infringements.size() + " airspace infringements");
        
        // Demonstrate finding shortest path
        System.out.println("\nFinding shortest path from WP4 to WP7:");
        List<TrajectorySegment> shortestPath = model.findShortestPath(wp4, wp7);
        if (shortestPath != null && !shortestPath.isEmpty()) {
            System.out.println("Shortest path has " + shortestPath.size() + " segments:");
            for (TrajectorySegment segment : shortestPath) {
                System.out.println("  " + segment.getSource().getId() + " -> " + 
                                  segment.getTarget().getId() + " (" + segment.getType() + ")");
            }
        } else {
            System.out.println("No path found");
        }
        
        // Demonstrate using the visitor pattern
        System.out.println("\nDemonstrating visitor pattern:");
        
        // Custom visitor that counts elements by type
        CountingVisitor counter = new CountingVisitor();
        
        // Visit different types of elements
        seg1.accept(counter);
        altSeg1.accept(counter);
        holdingSegment.accept(counter);
        reservedVolume.accept(counter);
        
        System.out.println("Element counts:");
        System.out.println("  Points: " + counter.getPointCount());
        System.out.println("  Lines: " + counter.getLineCount());
        System.out.println("  Polygons: " + counter.getPolygonCount());
        System.out.println("  Volumes: " + counter.getVolumeCount());
        System.out.println("  Trajectories: " + counter.getTrajectoryCount());
        
        // Example of using a finder visitor
        System.out.println("\nDemonstrating finder visitor:");
        FinderVisitor finder = new FinderVisitor("HOLD");
        
        for (SpatialElement element : elementsInBox) {
            element.accept(finder);
        }
        
        if (finder.getFoundElement() != null) {
            System.out.println("Found element with ID containing 'HOLD': " + 
                              finder.getFoundElement().getId());
        }
    }
}