package org.tash.extensions.reservation;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.visualization.AirspaceFeature;
import org.tash.extensions.visualization.AirspaceFeatureCollection;
import org.tash.extensions.visualization.AirspaceVisualizationService;

import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertEquals;

class CarfPolylineSplitMergeTest {
    @Test
    void splitsPolylineIntoContiguousSegmentsWithPerLegFixMetadata() {
        CarfReservationEvent event = routeEvent(Arrays.asList("FIXA", "FIXB", "FIXC"));

        List<AirspaceReservation> reservations = new CarfEventReservationMapper()
                .toReservations("RTE", event);

        assertEquals(2, reservations.size());

        AirspaceReservation first = reservations.get(0);
        assertEquals("FIXA", first.getRouteStartFix());
        assertEquals("FIXB", first.getRouteEndFix());
        assertEquals(Arrays.asList("FIXA", "FIXB"), first.getSourceFixes());
        assertEquals(0.0, first.getSourceRatioStart(), 0.0001);
        assertEquals(0.5, first.getSourceRatioEnd(), 0.0001);

        AirspaceReservation second = reservations.get(1);
        assertEquals("FIXB", second.getRouteStartFix());
        assertEquals("FIXC", second.getRouteEndFix());
        assertEquals(Arrays.asList("FIXB", "FIXC"), second.getSourceFixes());
        assertEquals(0.5, second.getSourceRatioStart(), 0.0001);
        assertEquals(1.0, second.getSourceRatioEnd(), 0.0001);
        assertEquals(first.getRouteEnd(), second.getRouteStart());
    }

    @Test
    void splitSegmentsRemainLosslesslyMergeableInVisualizationOutput() {
        List<AirspaceReservation> reservations = new CarfEventReservationMapper()
                .toReservations("RTE", routeEvent(Arrays.asList("FIXA", "FIXB", "FIXC")));

        AirspaceFeatureCollection collection = new AirspaceVisualizationService()
                .featuresForReservations(reservations);

        List<AirspaceFeature> routes = collection.getFeatures().stream()
                .filter(feature -> "flight-path".equals(feature.getProperties().get("featureKind")))
                .collect(Collectors.toList());

        assertEquals(2, routes.size());
        assertEquals("FIXA", routes.get(0).getProperties().get("routeStartFix"));
        assertEquals("FIXB", routes.get(0).getProperties().get("routeEndFix"));
        assertEquals(Arrays.asList("FIXA", "FIXB"), routes.get(0).getProperties().get("sourceFixes"));
        assertEquals(0.0, (Double) routes.get(0).getProperties().get("sourceRatioStart"), 0.0001);
        assertEquals(0.5, (Double) routes.get(0).getProperties().get("sourceRatioEnd"), 0.0001);
        assertEquals("ROUTE-A", routes.get(0).getProperties().get("polylineMergeKey"));
        assertEquals("FIXA->FIXB", routes.get(0).getProperties().get("polylineSegmentKey"));
        assertEquals(0, routes.get(0).getProperties().get("polylineSegmentIndex"));
        assertEquals("FIXB", routes.get(1).getProperties().get("routeStartFix"));
        assertEquals("FIXC", routes.get(1).getProperties().get("routeEndFix"));
        assertEquals(Arrays.asList("FIXB", "FIXC"), routes.get(1).getProperties().get("sourceFixes"));
        assertEquals(0.5, (Double) routes.get(1).getProperties().get("sourceRatioStart"), 0.0001);
        assertEquals(1.0, (Double) routes.get(1).getProperties().get("sourceRatioEnd"), 0.0001);
        assertEquals("ROUTE-A", routes.get(1).getProperties().get("polylineMergeKey"));
        assertEquals("FIXB->FIXC", routes.get(1).getProperties().get("polylineSegmentKey"));
        assertEquals(1, routes.get(1).getProperties().get("polylineSegmentIndex"));

        List<?> firstCoordinates = routes.get(0).getGeometry().getCoordinates();
        List<?> secondCoordinates = routes.get(1).getGeometry().getCoordinates();
        assertEquals(firstCoordinates.get(1), secondCoordinates.get(0));
    }

    @Test
    void fallsBackToPointIndexesWhenSourceFixNamesAreMissing() {
        CarfReservationEvent event = routeEvent(null);

        List<AirspaceReservation> reservations = new CarfEventReservationMapper()
                .toReservations("RTE", event);

        assertEquals(Arrays.asList("P0", "P1"), reservations.get(0).getSourceFixes());
        assertEquals("P0", reservations.get(0).getRouteStartFix());
        assertEquals("P1", reservations.get(0).getRouteEndFix());
        assertEquals(Arrays.asList("P1", "P2"), reservations.get(1).getSourceFixes());
        assertEquals("P1", reservations.get(1).getRouteStartFix());
        assertEquals("P2", reservations.get(1).getRouteEndFix());
    }

    private CarfReservationEvent routeEvent(List<String> sourceFixes) {
        return CarfReservationEvent.builder()
                .type(CarfReservationEventType.ROUTE_SEGMENT)
                .startTime(ZonedDateTime.parse("2026-01-01T00:00:00Z"))
                .endTime(ZonedDateTime.parse("2026-01-01T01:00:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .routeWidthNauticalMiles(20)
                .points(Arrays.asList(
                        point(39.0, -77.0, 24000),
                        point(40.0, -78.0, 25000),
                        point(41.0, -79.0, 26000)))
                .sourceFixes(sourceFixes)
                .routeGraphNodeIds(Arrays.asList("ROUTE-A"))
                .build();
    }

    private GeoCoordinate point(double lat, double lon, double altitude) {
        return GeoCoordinate.builder()
                .latitude(lat)
                .longitude(lon)
                .altitude(altitude)
                .build();
    }
}
