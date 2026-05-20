package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.reservation.RouteSeparationGeometry;

import static org.junit.jupiter.api.Assertions.*;

class RouteSeparationGeometryTest {
    private final RouteSeparationGeometry geometry = new RouteSeparationGeometry();

    @Test
    void computesCrossingRouteDistanceWithoutSamplingMisses() {
        double distance = geometry.segmentDistanceNauticalMiles(
                coordinate(0, 0),
                coordinate(0, 2),
                coordinate(-1, 1),
                coordinate(1, 1));

        assertEquals(0.0, distance, 0.0001);
    }

    @Test
    void computesParallelRouteDistanceInNauticalMiles() {
        double distance = geometry.segmentDistanceNauticalMiles(
                coordinate(20, -60),
                coordinate(20, -40),
                coordinate(18, -60),
                coordinate(18, -40));

        assertEquals(120.0, distance, 1.0);
    }

    @Test
    void computesLongitudinalGapAheadOfRouteSegment() {
        double gap = geometry.longitudinalGapNauticalMiles(
                coordinate(0, 0),
                coordinate(0, 1),
                coordinate(0, 2),
                coordinate(0, 3));

        assertEquals(60.0, gap, 0.5);
    }

    @Test
    void overlappingAlongTrackSegmentsHaveNoLongitudinalGap() {
        double gap = geometry.longitudinalGapNauticalMiles(
                coordinate(0, 0),
                coordinate(0, 2),
                coordinate(0, 1),
                coordinate(0, 3));

        assertEquals(0.0, gap, 0.0001);
    }

    private GeoCoordinate coordinate(double latitude, double longitude) {
        return GeoCoordinate.builder()
                .latitude(latitude)
                .longitude(longitude)
                .altitude(0)
                .build();
    }
}
