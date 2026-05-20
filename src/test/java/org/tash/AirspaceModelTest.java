package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.event.AirspaceInfringementEvent;
import org.tash.event.conflict.SeparationConflict;
import org.tash.flight.FlightTrajectory;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectoryType;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;

class AirspaceModelTest {
    @Test
    void detectsConflictsAcrossRegisteredFlightTrajectories() {
        AirspaceModel model = new AirspaceModel();
        ZonedDateTime start = ZonedDateTime.of(2026, 1, 1, 12, 0, 0, 0, ZoneOffset.UTC);
        ZonedDateTime end = start.plusMinutes(10);

        SpatialPoint west = model.addWaypoint(38.0, -77.20, 10000, "WEST");
        SpatialPoint east = model.addWaypoint(38.0, -77.00, 10000, "EAST");
        SpatialPoint south = model.addWaypoint(37.90, -77.10, 10000, "SOUTH");
        SpatialPoint north = model.addWaypoint(38.10, -77.10, 10000, "NORTH");

        FlightTrajectory flight1 = model.addFlightTrajectory("AAL100", "F-AAL100");
        FlightTrajectory flight2 = model.addFlightTrajectory("UAL200", "F-UAL200");
        flight1.addSegment(model.addLinearSegment(west, east, start, end, TrajectoryType.STANDARD, "AAL100-1"));
        flight2.addSegment(model.addLinearSegment(south, north, start, end, TrajectoryType.STANDARD, "UAL200-1"));

        AtomicInteger eventCount = new AtomicInteger();
        model.addEventListener("TRAJECTORY_CONFLICT", event -> eventCount.incrementAndGet());

        List<SeparationConflict> conflicts = model.checkTrajectoryConflicts("standard");

        assertFalse(conflicts.isEmpty());
        assertTrue(conflicts.stream().anyMatch(c -> c.getHorizontalSeparation() < AirspaceModel.MIN_HORIZONTAL_SEPARATION_NM));
        assertEquals(conflicts.size(), eventCount.get());
    }

    @Test
    void detectsTrajectoryEntryIntoReservedVolume() {
        AirspaceModel model = new AirspaceModel();
        ZonedDateTime start = ZonedDateTime.of(2026, 1, 1, 12, 0, 0, 0, ZoneOffset.UTC);

        SpatialPoint p1 = model.addWaypoint(38.0, -77.20, 10000, "P1");
        SpatialPoint p2 = model.addWaypoint(38.0, -77.00, 10000, "P2");
        FlightTrajectory flight = model.addFlightTrajectory("DAL300", "F-DAL300");
        LinearTrajectorySegment segment = model.addLinearSegment(p1, p2, start, start.plusMinutes(10), TrajectoryType.STANDARD, "DAL300-1");
        flight.addSegment(segment);

        SpatialVolume volume = model.reserveAirspace(Arrays.asList(
                model.addWaypoint(37.95, -77.15, 9000, "B1"),
                model.addWaypoint(38.05, -77.15, 9000, "B2"),
                model.addWaypoint(38.05, -77.05, 9000, "B3"),
                model.addWaypoint(37.95, -77.05, 9000, "B4")
        ), 9000, 11000, start.minusMinutes(5), start.plusMinutes(30), "RESERVED-DCA");

        List<AirspaceInfringementEvent> infringements = model.checkAirspaceInfringements();

        assertFalse(infringements.isEmpty());
        assertEquals(volume, infringements.get(0).getVolume());
        assertEquals(segment, infringements.get(0).getSegment());
    }
}
