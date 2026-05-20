package org.tash.extensions.visualization;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.time.ZonedDateTime;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class AirspaceVisualizationServiceTest {
    @Test
    void exportsReservationProtectedVolumeAsGeoJsonPolygon() {
        AirspaceReservation reservation = AirspaceReservation.builder()
                .id("R1")
                .reservationType("TIMING_TRIANGLE")
                .startTime(ZonedDateTime.parse("2010-01-01T00:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-01-01T01:00:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .protectedVolume(volume("V1"))
                .sourceFixes(Arrays.asList("A", "B", "C"))
                .routeWidthNauticalMiles(20)
                .build();

        AirspaceFeatureCollection collection =
                new AirspaceVisualizationService().featuresForReservations(Arrays.asList(reservation));

        assertEquals("FeatureCollection", collection.getType());
        assertEquals(1, collection.getFeatures().size());
        AirspaceFeature feature = collection.getFeatures().get(0);
        assertEquals("Feature", feature.getType());
        assertEquals("Polygon", feature.getGeometry().getType());
        assertEquals("reservation", feature.getProperties().get("featureKind"));
        assertEquals("TIMING_TRIANGLE", feature.getProperties().get("reservationType"));
        assertNotNull(feature.getProperties().get("style"));
        assertTrue(new GeoJsonAirspaceExporter().toGeoJson(collection).contains("\"FeatureCollection\""));
    }

    @Test
    void exportsConflictExplanationMetadata() {
        AirspaceReservation first = AirspaceReservation.builder().id("A")
                .routeStart(point(30, -150, 24000)).routeEnd(point(30, -149, 24000))
                .lowerAltitudeFeet(24000).upperAltitudeFeet(26000).build();
        AirspaceReservation second = AirspaceReservation.builder().id("B")
                .routeStart(point(31, -150, 24000)).routeEnd(point(31, -149, 24000))
                .lowerAltitudeFeet(24000).upperAltitudeFeet(26000).build();
        ReservationConflict conflict = ReservationConflict.builder()
                .first(first)
                .second(second)
                .startTime(ZonedDateTime.parse("2010-01-01T00:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-01-01T00:10:00Z"))
                .firstStartPoint(first.getRouteStart())
                .secondStartPoint(second.getRouteStart())
                .minimumLateralDistanceNauticalMiles(10)
                .requiredLateralSeparationNauticalMiles(20)
                .requiredVerticalSeparationFeet(1000)
                .explanation("lateralDistance=10 verticalGap=0")
                .build();

        AirspaceFeature feature = new AirspaceVisualizationService()
                .featuresForConflicts(Arrays.asList(conflict))
                .getFeatures().get(0);

        assertEquals("LineString", feature.getGeometry().getType());
        assertEquals("conflict", feature.getProperties().get("featureKind"));
        assertEquals(10.0, (Double) feature.getProperties().get("minimumLateralDistanceNauticalMiles"), 0.0001);
        assertEquals("lateralDistance=10 verticalGap=0", feature.getProperties().get("explanation"));
    }

    private SpatialVolume volume(String id) {
        return SpatialVolume.builder()
                .id(id)
                .lowerAltitude(24000)
                .upperAltitude(26000)
                .startTime(ZonedDateTime.parse("2010-01-01T00:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-01-01T01:00:00Z"))
                .basePolygon(SpatialPolygon.builder()
                        .id(id + "-P")
                        .vertices(Arrays.asList(
                                spatialPoint("A", 30, -150),
                                spatialPoint("B", 31, -150),
                                spatialPoint("C", 30, -149)))
                        .build())
                .build();
    }

    private SpatialPoint spatialPoint(String id, double lat, double lon) {
        return SpatialPoint.builder().id(id).coordinate(point(lat, lon, 0)).build();
    }

    private GeoCoordinate point(double lat, double lon, double alt) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(alt).build();
    }
}
