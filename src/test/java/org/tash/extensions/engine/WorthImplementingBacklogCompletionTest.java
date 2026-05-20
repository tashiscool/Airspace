package org.tash.extensions.engine;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.feed.InMemoryOperationalFeedSource;
import org.tash.extensions.feed.OperationalFeedBatchResult;
import org.tash.extensions.feed.OperationalFeedEnvelope;
import org.tash.extensions.feed.OperationalFeedIngestService;
import org.tash.extensions.feed.OperationalFeedType;
import org.tash.extensions.ops.InMemoryOperationalMetricSink;
import org.tash.extensions.repository.InMemoryAuditReplayRepository;
import org.tash.extensions.repository.InMemoryOperationalDecisionRepository;
import org.tash.extensions.repository.InMemoryPirepReportRepository;
import org.tash.extensions.repository.InMemoryWeatherProductRepository;
import org.tash.extensions.repository.JsonFileAuditReplayRepository;
import org.tash.extensions.repository.JsonFileOperationalDecisionRepository;
import org.tash.extensions.repository.JsonFileWeatherProductRepository;
import org.tash.extensions.routing.OperationalRoutePlanRequest;
import org.tash.extensions.routing.OperationalRoutePlanResult;
import org.tash.extensions.routing.RoutePlanningConstraint;
import org.tash.extensions.routing.RouteReplanningService;
import org.tash.extensions.uncertainty.DecisionUncertaintyService;
import org.tash.extensions.uncertainty.ForecastUncertaintyModel;
import org.tash.extensions.uncertainty.PositionUncertaintyModel;
import org.tash.extensions.uncertainty.UncertaintyAssessmentRequest;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;
import org.tash.extensions.weather.avoid.CircularWeatherCell;
import org.tash.extensions.weather.avoid.PolygonalWeatherCell;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.product.WeatherConfidence;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductSource;
import org.tash.extensions.weather.product.WeatherProductType;
import org.tash.extensions.weather.product.WeatherValidityWindow;

import java.nio.file.Path;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class WorthImplementingBacklogCompletionTest {
    private static final ZonedDateTime T0 = ZonedDateTime.of(2026, 5, 20, 0, 0, 0, 0, ZoneOffset.UTC);

    @TempDir
    Path tempDir;

    @Test
    void feedIngestRoutesLocalSourcesIntoEngineParsers() {
        InMemoryOperationalFeedSource source = new InMemoryOperationalFeedSource("local-replay", Arrays.asList(
                envelope("wx-1", OperationalFeedType.WEATHER,
                        "METAR KJFK 200000Z 18012G22KT 1/2SM +TSRA BKN004 OVC010 18/16 A2992"),
                envelope("pirep-1", OperationalFeedType.PIREP,
                        "UA /TP B738 /OV 3000N15000W /TM 0015 /FL240 /TB SEV /RM CHOP"),
                envelope("notam-1", OperationalFeedType.NOTAM,
                        "NOTAMN Q) KZNY/QWALW/IV/NBO/W/000/180/3000N15000W005 A) KZNY "
                                + "B) 2605200000 C) 2605200600 E) AIRSPACE CLOSED F) SFC G) FL180")));

        OperationalFeedBatchResult batch = new OperationalFeedIngestService().ingest(source.poll());

        assertEquals(3, batch.getResults().size());
        assertEquals(3, batch.acceptedCount(), batch.getDiagnostics().toString());
        assertTrue(batch.getResults().stream().anyMatch(result -> result.getWeatherProductResult() != null));
        assertTrue(batch.getResults().stream().anyMatch(result -> result.getPirepResult() != null));
        assertTrue(batch.getResults().stream().anyMatch(result -> result.getNotamRestriction() != null));
        assertTrue(batch.getResults().stream().allMatch(result -> result.getRawPayloadHash() != null));
    }

    @Test
    void repositoriesPersistCanonicalJsonAndReloadArtifactText() {
        OperationalDecisionResult result = blockedDecision();
        JsonFileOperationalDecisionRepository decisionRepository =
                new JsonFileOperationalDecisionRepository(tempDir.resolve("decisions.json"));
        JsonFileAuditReplayRepository auditRepository =
                new JsonFileAuditReplayRepository(tempDir.resolve("audit.json"));
        JsonFileWeatherProductRepository weatherRepository =
                new JsonFileWeatherProductRepository(tempDir.resolve("weather.json"));

        String decisionId = decisionRepository.save(result);
        String auditId = auditRepository.saveAuditEnvelope(result.getAuditEnvelope());
        String replayId = auditRepository.saveReplayBundle(result.getReplayBundle());
        String weatherId = weatherRepository.save(result.getWeatherProducts().get(0));

        assertTrue(new JsonFileOperationalDecisionRepository(tempDir.resolve("decisions.json"))
                .findJsonById(decisionId).orElse("").contains("Route blocked"));
        assertTrue(new JsonFileAuditReplayRepository(tempDir.resolve("audit.json")).findJsonById(auditId).isPresent());
        assertTrue(new JsonFileAuditReplayRepository(tempDir.resolve("audit.json")).findJsonById(replayId).isPresent());
        assertTrue(new JsonFileWeatherProductRepository(tempDir.resolve("weather.json")).findJsonById(weatherId)
                .orElse("").contains("CONV"));
    }

    @Test
    void routeReplanningProducesCandidateThatAvoidsBlockingConstraint() {
        RoutePlanningConstraint constraint = RoutePlanningConstraint.builder()
                .id("wx-poly")
                .type(OperationalConstraintType.WEATHER_HAZARD)
                .geometry(blockingPolygon())
                .rationale("convective polygon crosses route")
                .build();

        OperationalRoutePlanResult result = new RouteReplanningService().plan(OperationalRoutePlanRequest.builder()
                .originalRoute(route())
                .constraints(Collections.singletonList(constraint))
                .corridorWidthNauticalMiles(5.0)
                .build());

        assertFalse(result.isBlocked(), result.getRationale());
        assertFalse(result.getCandidates().isEmpty());
        assertTrue(result.getCandidates().get(0).getAvoidedConstraintIds().contains("wx-poly"));
    }

    @Test
    void uncertaintyLowersConfidenceAndEngineAttachesAssessmentAndRoutePlan() {
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .route(route())
                .weatherProducts(Collections.singletonList(convectiveProduct()))
                .uncertaintyRequest(UncertaintyAssessmentRequest.builder()
                        .positionUncertainty(PositionUncertaintyModel.builder()
                                .horizontalNauticalMiles(18.0)
                                .verticalFeet(2000.0)
                                .confidence(0.8)
                                .build())
                        .forecastUncertainty(ForecastUncertaintyModel.builder()
                                .forecastSpreadNauticalMiles(25.0)
                                .confidencePenalty(0.05)
                                .source("ensemble-spread")
                                .build())
                        .build())
                .build());

        assertEquals(WeatherDecisionAction.BLOCKED, result.getAction());
        assertNotNull(result.getUncertaintyAssessment());
        assertTrue(result.getConfidence() < 1.0);
        assertNotNull(result.getRoutePlanResult());
        assertTrue(result.getTrace().getSteps().stream().anyMatch(step -> "uncertainty".equals(step.getStage())));
    }

    @Test
    void runtimeOrchestratesFeedDecisionPersistenceAndMetrics() {
        InMemoryOperationalMetricSink metrics = new InMemoryOperationalMetricSink();
        InMemoryOperationalDecisionRepository decisionRepository = new InMemoryOperationalDecisionRepository();
        InMemoryAuditReplayRepository auditRepository = new InMemoryAuditReplayRepository();
        InMemoryWeatherProductRepository weatherRepository = new InMemoryWeatherProductRepository();
        InMemoryPirepReportRepository pirepRepository = new InMemoryPirepReportRepository();
        InMemoryOperationalFeedSource source = new InMemoryOperationalFeedSource("runtime-feed",
                Collections.singletonList(envelope("wx-1", OperationalFeedType.WEATHER,
                        "NEXRAD CWAP 3000N15000W 3000N14900W 3100N14900W TOPS FL420 GROWTH CONF 95")));

        OperationalRuntimeResult result = new OperationalRuntimeService().run(OperationalRuntimeRequest.builder()
                .feedSources(Collections.singletonList(source))
                .decisionRequest(OperationalDecisionRequest.builder()
                        .decisionTime(T0)
                        .route(route())
                        .weatherProducts(Collections.singletonList(convectiveProduct()))
                        .build())
                .decisionRepository(decisionRepository)
                .auditReplayRepository(auditRepository)
                .weatherProductRepository(weatherRepository)
                .pirepReportRepository(pirepRepository)
                .metricSink(metrics)
                .build());

        assertTrue(result.isAccepted());
        assertFalse(result.getPersistedArtifactIds().isEmpty());
        assertFalse(decisionRepository.listIds().isEmpty());
        assertFalse(auditRepository.listIds().isEmpty());
        assertFalse(weatherRepository.listIds().isEmpty());
        assertTrue(metrics.sum("feed.messages.received") >= 1.0);
        assertTrue(metrics.sum("decision.evaluated") >= 1.0);
        assertEquals(WeatherDecisionAction.BLOCKED, result.getDecisionResult().getAction());
    }

    @Test
    void decisionUncertaintyServicePreservesDeterminismWhenNoInputsSupplied() {
        assertEquals(0.72, new DecisionUncertaintyService().assess(UncertaintyAssessmentRequest.builder()
                .baseConfidence(0.72)
                .build()).getAdjustedConfidence(), 0.0001);
    }

    private OperationalDecisionResult blockedDecision() {
        return new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .route(route())
                .weatherProducts(Collections.singletonList(convectiveProduct()))
                .build());
    }

    private OperationalFeedEnvelope envelope(String id, OperationalFeedType type, String raw) {
        return OperationalFeedEnvelope.builder()
                .id(id)
                .sourceId("test")
                .type(type)
                .receivedAt(T0)
                .rawPayload(raw)
                .build();
    }

    private WeatherProduct convectiveProduct() {
        CircularWeatherCell cell = CircularWeatherCell.builder()
                .id("CONV")
                .type(WeatherElementType.CONVECTION)
                .severity(HazardSeverity.EXTREME)
                .startTime(T0)
                .endTime(T0.plusHours(4))
                .minAltitude(0)
                .maxAltitude(45000)
                .center(point(30.0, -150.0))
                .radius(80.0)
                .build();
        return WeatherProduct.builder()
                .id("CONV")
                .type(WeatherProductType.NEXRAD_POLYGON)
                .source(WeatherProductSource.NWS)
                .sourceProduct("CWAP")
                .provider("test")
                .rawText("CWAP CONV")
                .validity(WeatherValidityWindow.builder().validStart(T0).validEnd(T0.plusHours(4)).build())
                .issuedAt(T0)
                .receivedAt(T0)
                .forecastHour(0)
                .confidence(WeatherConfidence.builder().value(0.95).basis("test").build())
                .provenance("test")
                .hazard(cell)
                .geometry(blockingPolygon())
                .lowerAltitudeFeet(0.0)
                .upperAltitudeFeet(45000.0)
                .echoTopFeet(42000.0)
                .growthTrend(1.0)
                .stormPhase("growing")
                .build();
    }

    private List<GeoCoordinate> route() {
        return Arrays.asList(point(30.0, -150.5), point(30.0, -148.5));
    }

    private List<GeoCoordinate> blockingPolygon() {
        return Arrays.asList(
                point(29.5, -150.1),
                point(30.5, -150.1),
                point(30.5, -149.5),
                point(29.5, -149.5));
    }

    private GeoCoordinate point(double lat, double lon) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(24000).build();
    }
}
