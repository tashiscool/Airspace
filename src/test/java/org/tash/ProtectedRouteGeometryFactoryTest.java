package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ProtectedRouteGeometryFactory;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class ProtectedRouteGeometryFactoryTest {
    private final ProtectedRouteGeometryFactory factory = new ProtectedRouteGeometryFactory();

    @Test
    void createsGeodeticStripAroundRouteCenterline() {
        GeoCoordinate start = coordinate(30, -150, 24000);
        GeoCoordinate end = coordinate(30, -149, 24000);

        SpatialPolygon strip = factory.stripPolygon("ROUTE", start, end, 10);

        assertEquals(4, strip.getVertices().size());
        assertEquals(10, start.distanceTo(strip.getVertices().get(0).getCoordinate()), 0.05);
        assertEquals(10, end.distanceTo(strip.getVertices().get(1).getCoordinate()), 0.05);
    }

    @Test
    void createsCapsuleWithSemicircularEndCaps() {
        GeoCoordinate start = coordinate(30, -150, 24000);
        GeoCoordinate end = coordinate(30, -149, 24000);

        SpatialPolygon capsule = factory.capsulePolygon("ROUTE", start, end, 10, 6);

        assertEquals(14, capsule.getVertices().size());
        assertEquals(10, start.distanceTo(capsule.getVertices().get(0).getCoordinate()), 0.05);
        assertEquals(10, end.distanceTo(capsule.getVertices().get(7).getCoordinate()), 0.05);
        assertTrue(capsule.getBoundingBox().getMinLat() < 30);
        assertTrue(capsule.getBoundingBox().getMaxLat() > 30);
    }

    @Test
    void createsRouteSegmentVolumeFromReservationMetadata() {
        AirspaceReservation reservation = AirspaceReservation.builder()
                .id("R1")
                .startTime(ZonedDateTime.parse("2010-03-02T12:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-03-02T13:00:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .deconflictionLowerAltitudeFeet(23750.5)
                .deconflictionUpperAltitudeFeet(26249.5)
                .routeWidthNauticalMiles(20)
                .avanaMinutes(60)
                .routeStart(coordinate(30, -150, 24000))
                .routeEnd(coordinate(30, -149, 24000))
                .build();

        SpatialVolume volume = factory.routeSegmentVolume("R1", reservation);

        assertNotNull(volume);
        assertEquals(23750.5, volume.getLowerAltitude(), 0.0001);
        assertEquals(26249.5, volume.getUpperAltitude(), 0.0001);
        assertEquals(ZonedDateTime.parse("2010-03-02T14:00:00Z"), volume.getEndTime());
        assertTrue(volume.getBasePolygon().getVertices().size() > 4);
    }

    @Test
    void padsAreaPolygonsOutwardForProtectedAirspace() {
        List<GeoCoordinate> triangle = Arrays.asList(
                coordinate(30.0, -150.0, 24000),
                coordinate(30.5, -149.5, 24000),
                coordinate(30.0, -149.0, 24000));

        SpatialPolygon original = factory.paddedPolygon("AREA", triangle, 0);
        SpatialPolygon padded = factory.paddedPolygon("AREA", triangle, 10);
        GeoCoordinate center = original.getSamplePoint().getCoordinate();

        assertEquals(3, padded.getVertices().size());
        for (int i = 0; i < triangle.size(); i++) {
            double originalDistance = center.distanceTo(original.getVertices().get(i).getCoordinate());
            double paddedDistance = center.distanceTo(padded.getVertices().get(i).getCoordinate());
            assertEquals(originalDistance + 10, paddedDistance, 0.2);
        }
    }

    private GeoCoordinate coordinate(double latitude, double longitude, double altitude) {
        return GeoCoordinate.builder()
                .latitude(latitude)
                .longitude(longitude)
                .altitude(altitude)
                .build();
    }
}
