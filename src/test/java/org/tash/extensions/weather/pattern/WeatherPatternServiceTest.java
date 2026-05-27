package org.tash.extensions.weather.pattern;

import org.junit.jupiter.api.Test;
import org.tash.extensions.feed.InMemoryOperationalFeedSource;
import org.tash.extensions.feed.OperationalFeedBatchResult;
import org.tash.extensions.feed.OperationalFeedEnvelope;
import org.tash.extensions.feed.OperationalFeedIngestService;
import org.tash.extensions.feed.OperationalFeedType;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class WeatherPatternServiceTest {
    private final WeatherPatternService service = new WeatherPatternService();

    @Test
    void awcGeoJsonMetadataMapsIntoWeatherPatternWithoutLosingSourceRefs() {
        OperationalFeedBatchResult batch = ingest(envelope("SIGMET CONV SEV VALID 201200/201800 3000N15000W 3000N14900W 3100N14900W FL240-260",
                OperationalFeedType.WEATHER,
                metadata("airsigmet", "{\"type\":\"Polygon\",\"coordinates\":[[[-150,30,0],[-149,30,0],[-149,31,0],[-150,30,0]]]}")));

        List<WeatherPattern> patterns = service.patternsFromFeedResults(batch.getResults());

        assertFalse(patterns.isEmpty());
        WeatherPattern pattern = patterns.get(0);
        assertEquals(WeatherPatternType.CONVECTION, pattern.getType());
        assertEquals("POLYGON", pattern.getGeometryIntent());
        assertFalse(pattern.getGeometry().isEmpty());
        assertTrue(pattern.getSourceRefs().stream().anyMatch(ref -> ref.startsWith("AWC:")));
        assertEquals("https://aviationweather.gov/api/data/airsigmet?format=geojson", pattern.getSourceUrl());
    }

    @Test
    void metarWithoutCoordinatesRemainsStationGuidanceWithoutFakeGeometry() {
        OperationalFeedBatchResult batch = ingest(envelope("METAR KJFK 200000Z 18012KT 1/2SM TSRA BKN004",
                OperationalFeedType.WEATHER,
                metadata("metar", null)));

        WeatherPattern pattern = service.patternsFromFeedResults(batch.getResults()).get(0);

        assertEquals(WeatherPatternType.CEILING_VISIBILITY, pattern.getType());
        assertEquals("STATION_GUIDANCE", pattern.getGeometryIntent());
        assertTrue(pattern.getGeometry().isEmpty());
        assertTrue(pattern.getDiagnostics().stream().anyMatch(d -> d.contains("MISSING_GEOMETRY")));
    }

    @Test
    void routeSamplingDetectsTimeAltitudeAndGeometryOverlap() {
        OperationalFeedBatchResult batch = ingest(envelope("SIGMET CONV SEV VALID 201200/201800 3000N15000W 3000N14900W 3100N14900W FL240-260",
                OperationalFeedType.WEATHER,
                metadata("airsigmet", "{\"type\":\"Polygon\",\"coordinates\":[[[-150,30,0],[-149,30,0],[-149,31,0],[-150,30,0]]]}")));
        WeatherPatternQuery query = new WeatherPatternQuery();
        query.setRoute(Arrays.asList(Arrays.asList(30.0, -150.2, 25000.0), Arrays.asList(30.5, -148.8, 25000.0)));
        query.setLowerAltitudeFeet(24000.0);
        query.setUpperAltitudeFeet(26000.0);
        query.setCorridorNauticalMiles(40.0);

        List<RouteWeatherPatternIntersection> intersections =
                service.routeSample(query, service.patternsFromFeedResults(batch.getResults()));

        assertEquals(1, intersections.size());
        assertTrue(intersections.get(0).isGeometryOverlap());
        assertTrue(intersections.get(0).isAltitudeOverlap());
        assertTrue(intersections.get(0).isTimeOverlap());
        assertEquals(WeatherPatternType.CONVECTION, intersections.get(0).getPatternType());
    }

    @Test
    void eventsGroupPatternsByWeatherFamily() {
        OperationalFeedBatchResult batch = ingest(
                envelope("SIGMET CONV SEV 3000N15000W 3000N14900W 3100N14900W FL240-260",
                        OperationalFeedType.WEATHER,
                        metadata("airsigmet", "{\"type\":\"Polygon\",\"coordinates\":[[[-150,30,0],[-149,30,0],[-149,31,0],[-150,30,0]]]}")),
                envelope("UA /OV JFK090020 /TM 2000 /FL240 /TB MOD",
                        OperationalFeedType.PIREP,
                        metadata("pirep", "{\"type\":\"Point\",\"coordinates\":[-149.5,30.2,24000]}")));

        List<WeatherPatternEvent> events = service.events(service.patternsFromFeedResults(batch.getResults()), Arrays.asList("AWC:airsigmet-1"));

        assertTrue(events.stream().anyMatch(event -> event.getType() == WeatherPatternType.CONVECTION));
        assertTrue(events.stream().anyMatch(event -> event.getType() == WeatherPatternType.PIREP_CLUSTER));
    }

    private OperationalFeedBatchResult ingest(OperationalFeedEnvelope... envelopes) {
        return new OperationalFeedIngestService().ingest(new InMemoryOperationalFeedSource("test-weather",
                Arrays.asList(envelopes)).poll());
    }

    private OperationalFeedEnvelope envelope(String raw, OperationalFeedType type, Map<String, String> metadata) {
        return OperationalFeedEnvelope.builder()
                .id("env-" + Math.abs(raw.hashCode()))
                .sourceId("test-weather")
                .type(type)
                .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .rawPayload(raw)
                .metadata(metadata)
                .build();
    }

    private Map<String, String> metadata(String product, String geometryJson) {
        Map<String, String> values = new LinkedHashMap<>();
        values.put("sourceUrl", "https://aviationweather.gov/api/data/" + product + "?format=geojson");
        values.put("productFamily", product.toUpperCase());
        values.put("sourceRef", "AWC:" + product + "-1");
        if (geometryJson != null) {
            values.put("geoJsonGeometry", geometryJson);
        }
        return values;
    }
}
