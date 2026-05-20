package org.tash.extensions.engine;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.refdata.InMemoryCarfReferenceDataProvider;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.messaging.UsnsIngestResult;
import org.tash.extensions.messaging.UsnsIngestService;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;
import org.tash.extensions.weather.avoid.CircularWeatherCell;
import org.tash.extensions.weather.decision.RouteWeatherDecisionRequest;
import org.tash.extensions.weather.decision.RouteWeatherImpactResult;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;
import org.tash.extensions.weather.decision.WeatherRouteImpactModel;
import org.tash.extensions.weather.pirep.PirepDiagnosticType;
import org.tash.extensions.weather.product.WeatherConfidence;
import org.tash.extensions.weather.product.WeatherMovementVector;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductParser;
import org.tash.extensions.weather.product.WeatherProductSource;
import org.tash.extensions.weather.product.WeatherProductType;
import org.tash.extensions.weather.product.WeatherValidityWindow;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class OperationalDecisionEngineTest {
    private static final ZonedDateTime T0 = ZonedDateTime.parse("2026-05-20T00:00:00Z");

    @Test
    void usnsEnvelopeRoutesWeatherFamiliesIntoEngineArtifacts() {
        String raw = envelope("PIREP UA /TP B738 /OV 3000N15000W /FL240 /TB SEV\n"
                + "SIGMET CONV SEV 3000N15000W 3000N14900W 3100N14900W FL240-260 CONF 90");

        UsnsIngestResult result = new UsnsIngestService().parse(raw);

        assertTrue(result.getErrors().isEmpty(), result.getErrors().toString());
        assertTrue(result.getTransactionResults().getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.PIREP));
        assertTrue(result.getTransactionResults().getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.SIGMET));
        assertFalse(result.getPirepResults().isEmpty());
        assertFalse(result.getWeatherProducts().isEmpty());
    }

    @Test
    void weatherParserExtractsPirepSigmetMetarAndPolygonAdvisory() {
        WeatherProductParser parser = new WeatherProductParser();
        WeatherProductParseResult pirep = parser.parse("PIREP UA /TP B738 /OV 3000N15000W /FL240 /IC SEV /RM TRACE ICE", null);
        WeatherProductParseResult sigmet = parser.parse("SIGMET CONV SEV 3000N15000W 3000N14900W 3100N14900W FL240-260 CONF 90", null);
        WeatherProductParseResult metar = parser.parse("METAR KJFK CIG 004 VIS 1/2 3000N15000W", null);
        WeatherProductParseResult nexrad = parser.parse("NEXRAD CWAP 3000N15000W 3000N14900W 3100N14900W MOV 180 40 CONF 80 TOPS FL420 GROWTH", null);

        assertTrue(pirep.isAccepted());
        assertNotNull(pirep.getPirepReport());
        assertFalse(pirep.getSourceSpans().isEmpty());
        assertEquals(WeatherProductType.SIGMET, sigmet.getClassifiedType());
        assertNotNull(sigmet.getProduct().getHazard());
        assertEquals(WeatherProductType.METAR, metar.getClassifiedType());
        assertEquals(WeatherProductType.NEXRAD_POLYGON, nexrad.getClassifiedType());
        assertNotNull(nexrad.getProduct().getMovement());
        assertEquals(42000.0, nexrad.getProduct().getEchoTopFeet());
        assertEquals(1.0, nexrad.getProduct().getGrowthTrend());
    }

    @Test
    void weatherParserHandlesAviationFieldGrammarAndValidity() {
        WeatherProductParser parser = new WeatherProductParser();
        WeatherProductParseResult pirep = parser.parse("UUA /TP A320 /OV 30N150W /TM 0035 /FL240 /TB MOD-SEV /RM CHOP", null);
        WeatherProductParseResult airmet = parser.parse("AIRMET ZULU VALID 200000/200600 ICE MOD FROM N3000 W15000 TO N3100 W14900 FL180-240 MOV 090 25", null);
        WeatherProductParseResult sfcSigmet = parser.parse("SIGMET CONV VALID 200000/200600 FROM 3000N15000W TO 3000N14900W TO 3100N14900W SFC-FL240", null);
        WeatherProductParseResult taf = parser.parse("TAF KJFK 200000Z 2000/2106 18012G24KT 1/2SM +TSRA BKN004 OVC008", null);
        WeatherProductParseResult tafChanging = parser.parse("TAF KJFK 200000Z 2000/2106 2SM RA BKN010 FM200300 1/2SM TSRA BKN004 TEMPO 2004/2008 1/4SM FG VV002 PROB30 2008/2012 3SM RA BKN012", null);
        WeatherProductParseResult speci = parser.parse("SPECI KJFK 200000Z 12008KT M1/4SM R04/1200FT FG VV002 18/12 A2992", null);
        WeatherProductParseResult convective = parser.parse("CONVECTIVE SIGMET 1E VALID UNTIL 200600Z FROM 30N150W TO 31N149W TOP FL450 MOV NE 25KT", null);
        WeatherProductParseResult cwa = parser.parse("CWA ZNY WEATHER ADVISORY HOLD FOR METEOROLOGIST REVIEW", null);
        WeatherProductParseResult pirepWx = parser.parse("UA /TP C172 /OV 3000N15000W /TM 0010 /FL080 /WX TSRA /RM BUILDUPS", null);
        WeatherProductParseResult lineAirmet = parser.parse("AIRMET TANGO VALID 200000/200600 TURB FROM 3000N15000W TO 3100N14900W WI 20 NM EITHER SIDE FL180-240", null);

        assertTrue(pirep.isAccepted(), pirep.getErrors().toString());
        assertEquals(35, pirep.getPirepReport().getObservationTime().getMinute());
        assertTrue(airmet.isAccepted(), airmet.getErrors().toString());
        assertEquals(WeatherProductType.AIRMET, airmet.getClassifiedType());
        assertEquals(T0, airmet.getProduct().getValidity().getValidStart());
        assertEquals(T0.plusHours(6), airmet.getProduct().getValidity().getValidEnd());
        assertEquals(0.0, sfcSigmet.getProduct().getLowerAltitudeFeet());
        assertEquals(24000.0, sfcSigmet.getProduct().getUpperAltitudeFeet());
        assertEquals(WeatherProductType.TAF, taf.getClassifiedType());
        assertEquals("KJFK", taf.getProduct().getStationId());
        assertEquals(T0, taf.getProduct().getIssuedAt());
        assertEquals(400.0, taf.getProduct().getCeilingFeet());
        assertEquals(0.5, taf.getProduct().getVisibilityStatuteMiles());
        assertEquals(180, taf.getProduct().getWindDirectionDegrees());
        assertEquals(12.0, taf.getProduct().getWindSpeedKnots());
        assertEquals(24.0, taf.getProduct().getWindGustKnots());
        assertTrue(taf.getProduct().getWeatherPhenomena().contains("TSRA"));
        assertTrue(tafChanging.getProduct().getForecastChangeGroups().contains("FM200300"));
        assertTrue(tafChanging.getProduct().getForecastChangeGroups().stream().anyMatch(group -> group.startsWith("TEMPO")));
        assertTrue(tafChanging.getProduct().getForecastChangeGroups().stream().anyMatch(group -> group.startsWith("PROB30")));
        assertEquals(4, tafChanging.getProduct().getForecastSlices().size());
        assertEquals("FM", tafChanging.getProduct().getForecastSlices().get(1).getGroupType());
        assertEquals(0.30, tafChanging.getProduct().getForecastSlices().get(3).getConfidence(), 0.001);
        assertEquals(WeatherProductType.METAR, speci.getClassifiedType());
        assertEquals("KJFK", speci.getProduct().getStationId());
        assertEquals(0.25, speci.getProduct().getVisibilityStatuteMiles());
        assertEquals(200.0, speci.getProduct().getCeilingFeet());
        assertEquals(1200.0, speci.getProduct().getRunwayVisualRangeFeet());
        assertEquals(18.0, speci.getProduct().getTemperatureCelsius());
        assertEquals(12.0, speci.getProduct().getDewpointCelsius());
        assertEquals(29.92, speci.getProduct().getAltimeterInchesHg(), 0.001);
        assertTrue(speci.getProduct().getWeatherPhenomena().contains("FG"));
        assertEquals(WeatherProductType.SIGMET, convective.getClassifiedType());
        assertEquals(T0.plusHours(6), convective.getProduct().getValidity().getValidEnd());
        assertEquals(45.0, convective.getProduct().getMovement().getBearingDegrees());
        assertEquals(45000.0, convective.getProduct().getEchoTopFeet());
        assertTrue(cwa.isClassifiedOnly());
        assertTrue(pirepWx.isAccepted(), pirepWx.getErrors().toString());
        assertEquals(40.0, lineAirmet.getProduct().getLineWidthNauticalMiles());
        assertTrue(lineAirmet.getProduct().getHazard() instanceof org.tash.extensions.weather.avoid.PolygonalWeatherCell);
    }

    @Test
    void weatherParserReturnsDiagnosticsAndClassifiedOnlyFallbacks() {
        WeatherProductParser parser = new WeatherProductParser();
        WeatherProductParseResult malformedPirep = parser.parse("PIREP UA /TP B738 /TB SEV", null);
        WeatherProductParseResult radialPirep = parser.parse("UA /TP B738 /OV JFK090020 /TM 0015 /FL240 /TB MOD", null);
        WeatherProductParseResult retained = parser.parse("WEATHER DESK NOTE HOLD FOR FORECAST REVIEW", null);

        assertFalse(malformedPirep.isAccepted());
        assertFalse(malformedPirep.getDiagnostics().isEmpty());
        assertTrue(malformedPirep.getErrors().stream().anyMatch(e -> e.contains("PIREP missing")));
        assertFalse(radialPirep.isAccepted());
        assertEquals("JFK090020", radialPirep.getPirepReport().getLocationText());
        assertFalse(retained.isAccepted());
        assertTrue(retained.isClassifiedOnly());
    }

    @Test
    void staleWeatherProductsCreateWarningTraceAndMonitorDecision() {
        WeatherProduct stale = weatherProduct("STALE", HazardSeverity.LIGHT).toBuilder()
                .issuedAt(T0.minusHours(2))
                .receivedAt(T0.minusHours(2))
                .build();
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .route(route())
                .weatherProducts(Arrays.asList(stale))
                .build());

        assertTrue(result.getTrace().getSteps().stream().anyMatch(step -> "warning".equals(step.getStage())));
        assertFalse(result.getWeatherDataWarnings().isEmpty());
    }

    @Test
    void operationalEngineFusesCarfRouteWeatherAndTraceIntoBlockedDecision() {
        WeatherProduct convectiveLine = weatherProduct("CONV-LINE", HazardSeverity.EXTREME);
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .rawCarfMessages(Arrays.asList(carf()))
                .weatherProducts(Arrays.asList(convectiveLine))
                .route(route())
                .build());

        assertEquals(WeatherDecisionAction.BLOCKED, result.getAction());
        assertEquals(WeatherRecommendedAction.ROUTE_BLOCKED, result.getRecommendedAction());
        assertFalse(result.getReservations().isEmpty());
        assertFalse(result.getRouteBlockages().isEmpty());
        assertTrue(result.getConstraints().stream().anyMatch(c -> c.getType() == OperationalConstraintType.ROUTE_BLOCKAGE));
        assertTrue(result.getTrace().getSteps().stream().anyMatch(s -> "parse".equals(s.getStage()) || "map".equals(s.getStage())));
        assertTrue(result.getTrace().getSteps().stream().anyMatch(s -> "action".equals(s.getStage())));
        assertTrue(result.getTrace().getSteps().stream().anyMatch(s -> "rule-evaluation".equals(s.getStage())
                && s.getRule() != null
                && DecisionRuleCatalog.OP_ACTION_PRECEDENCE.getId().equals(s.getRule().getId())
                && !s.getThresholds().isEmpty()
                && s.getConfidenceMath() != null));
        assertTrue(result.getDecisionSummary().contains("BLOCKED"));
        assertFalse(result.getBlockingConstraints().isEmpty());
    }

    @Test
    void fusionCreatesCompoundRiskForOverlappingNotamAndWeather() {
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .route(route())
                .weatherProducts(Arrays.asList(weatherProduct("CONV-LINE", HazardSeverity.SEVERE)))
                .notamRestrictions(Arrays.asList(NotamAirspaceRestriction.builder()
                        .id("NOTAM-1")
                        .effectiveStart(T0.minusMinutes(10))
                        .effectiveEnd(T0.plusHours(2))
                        .lowerAltitudeFeet(20000)
                        .upperAltitudeFeet(30000)
                        .centerLatitude(30)
                        .centerLongitude(-149.5)
                        .radiusNauticalMiles(40)
                        .description("Temporary flight restriction over convective route segment")
                        .build()))
                .build());

        assertTrue(result.getConstraints().stream()
                .anyMatch(c -> c.getType() == OperationalConstraintType.COMPOUND_OPERATIONAL_RISK));
        OperationalConstraint compound = result.getConstraints().stream()
                .filter(c -> c.getType() == OperationalConstraintType.COMPOUND_OPERATIONAL_RISK)
                .findFirst()
                .orElseThrow(AssertionError::new);
        assertTrue(compound.getOverlapDimensions().containsAll(Arrays.asList("time", "altitude", "geometry")));
        assertEquals(2, compound.getComponentConstraintIds().size());
        assertTrue(result.getConstraintCountsByType().get(OperationalConstraintType.COMPOUND_OPERATIONAL_RISK) > 0);
    }

    @Test
    void fusionUsesGeometryIntersectionBeyondSimpleBoundingBoxes() {
        OperationalConstraint left = OperationalConstraint.builder()
                .id("left")
                .type(OperationalConstraintType.WEATHER_HAZARD)
                .startTime(T0)
                .endTime(T0.plusHours(1))
                .lowerAltitudeFeet(0)
                .upperAltitudeFeet(40000)
                .severity(org.tash.extensions.weather.decision.WeatherDecisionSeverity.WARNING)
                .confidence(0.9)
                .geometry(Arrays.asList(point(0, 0, 0), point(2, 2, 0)))
                .build();
        OperationalConstraint right = OperationalConstraint.builder()
                .id("right")
                .type(OperationalConstraintType.NOTAM_RESTRICTION)
                .startTime(T0)
                .endTime(T0.plusHours(1))
                .lowerAltitudeFeet(0)
                .upperAltitudeFeet(40000)
                .severity(org.tash.extensions.weather.decision.WeatherDecisionSeverity.ADVISORY)
                .confidence(1.0)
                .geometry(Arrays.asList(point(0, 2, 0), point(2, 0, 0)))
                .build();
        OperationalConstraint bboxOnly = OperationalConstraint.builder()
                .id("bbox")
                .type(OperationalConstraintType.PIREP_HAZARD)
                .startTime(T0)
                .endTime(T0.plusHours(1))
                .lowerAltitudeFeet(0)
                .upperAltitudeFeet(40000)
                .severity(org.tash.extensions.weather.decision.WeatherDecisionSeverity.ADVISORY)
                .confidence(1.0)
                .geometry(Arrays.asList(point(1.8, 0.1, 0), point(1.9, 1.7, 0)))
                .build();

        ConstraintFusionResult intersecting = new ConstraintFusionService().fuse(ConstraintFusionRequest.builder()
                .weatherProducts(Collections.emptyList())
                .build());
        assertNotNull(intersecting);
        ConstraintFusionResult result = new ConstraintFusionService().fuse(ConstraintFusionRequest.builder()
                .routeBlockages(Collections.emptyList())
                .build());
        assertNotNull(result);

        List<OperationalConstraint> constraints = Arrays.asList(left, right, bboxOnly);
        ConstraintFusionResult fusion = new ConstraintFusionService().fuse(ConstraintFusionRequest.builder().build());
        assertNotNull(fusion);
        List<OperationalConstraint> compounds = invokeCompoundForTest(constraints);
        assertTrue(compounds.stream().anyMatch(c -> c.getComponentConstraintIds().containsAll(Arrays.asList("left", "right"))));
        assertFalse(compounds.stream().anyMatch(c -> c.getComponentConstraintIds().containsAll(Arrays.asList("left", "bbox"))));
    }

    @Test
    void routeImpactModelProducesProbabilityMetricsForExtremeWeather() {
        RouteWeatherImpactResult result = new WeatherRouteImpactModel().evaluate(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .forecastHorizon(Duration.ofHours(1))
                .forecastStep(Duration.ofHours(1))
                .products(Arrays.asList(weatherProduct("WX", HazardSeverity.EXTREME)))
                .build());

        assertTrue(result.getMaximumBlockedProbability() > 0.9);
        assertTrue(result.getMaximumCapacityImpact() > 0.9);
        assertTrue(result.getMaximumDeviationLikelihood() > 0.9);
        assertFalse(result.getPredictions().get(0).getSegmentBlockedProbabilities().isEmpty());
        assertFalse(result.getPredictions().get(0).getRuleIds().isEmpty());
        assertNotNull(result.getPredictions().get(0).getConfidenceMath());
    }

    @Test
    void routeImpactScoringAccountsForGrowthDecayEchoTopsAndLeadTime() {
        WeatherProduct growing = weatherProduct("GROWING", HazardSeverity.SEVERE, 42000.0, 1.0, T0.plusHours(10));
        WeatherProduct decaying = weatherProduct("DECAYING", HazardSeverity.SEVERE, 42000.0, -1.0, T0.plusHours(10));
        WeatherProduct belowRouteEchoTops = weatherProduct("LOW-TOPS", HazardSeverity.EXTREME, 10000.0, 0.0, T0.plusHours(10));

        RouteWeatherImpactResult growingImpact = new WeatherRouteImpactModel().evaluate(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .forecastHorizon(Duration.ofHours(8))
                .forecastStep(Duration.ofHours(8))
                .products(Arrays.asList(growing))
                .build());
        RouteWeatherImpactResult decayingImpact = new WeatherRouteImpactModel().evaluate(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .forecastHorizon(Duration.ofHours(1))
                .forecastStep(Duration.ofHours(1))
                .products(Arrays.asList(decaying))
                .build());
        RouteWeatherImpactResult lowTopsImpact = new WeatherRouteImpactModel().evaluate(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .forecastHorizon(Duration.ofHours(1))
                .forecastStep(Duration.ofHours(1))
                .products(Arrays.asList(belowRouteEchoTops))
                .build());

        assertTrue(growingImpact.getMaximumBlockedProbability() > decayingImpact.getMaximumBlockedProbability());
        assertTrue(lowTopsImpact.getMaximumBlockedProbability() < 0.4);
        assertTrue(growingImpact.getPredictions().get(0).getConfidence()
                > growingImpact.getPredictions().get(1).getConfidence());
    }

    @Test
    void constraintIndexFiltersByCorridorTimeAndAltitude() {
        OperationalConstraint onRoute = OperationalConstraint.builder()
                .id("on")
                .type(OperationalConstraintType.WEATHER_HAZARD)
                .startTime(T0)
                .endTime(T0.plusHours(1))
                .lowerAltitudeFeet(20000)
                .upperAltitudeFeet(30000)
                .geometry(route())
                .build();
        OperationalConstraint farAway = OperationalConstraint.builder()
                .id("far")
                .type(OperationalConstraintType.NOTAM_RESTRICTION)
                .startTime(T0)
                .endTime(T0.plusHours(1))
                .lowerAltitudeFeet(20000)
                .upperAltitudeFeet(30000)
                .geometry(Arrays.asList(point(40, -120, 25000), point(41, -120, 25000)))
                .build();

        ConstraintIndex index = new ConstraintIndex(Arrays.asList(onRoute, farAway));
        List<OperationalConstraint> matches = index.query(
                RouteCorridor.builder().points(route()).bufferNauticalMiles(10).build(),
                TimeWindow.builder().start(T0.minusMinutes(5)).end(T0.plusHours(2)).build(),
                AltitudeBand.builder().lowerFeet(24000).upperFeet(26000).build());

        assertEquals(Arrays.asList("on"), matches.stream().map(OperationalConstraint::getId).collect(Collectors.toList()));
        assertTrue(index.getGridCellCount() > 0);
        assertEquals(Arrays.asList(0), new java.util.ArrayList<>(index.queryRouteSegments(route(),
                        TimeWindow.builder().start(T0.minusMinutes(5)).end(T0.plusHours(2)).build(),
                        AltitudeBand.builder().lowerFeet(24000).upperFeet(26000).build(),
                        10).keySet()));
    }

    @Test
    void usnsMalformedRequiredWeatherRejectsBatchWithDiagnostics() {
        UsnsIngestResult result = new UsnsIngestService().parse(envelope("PIREP UA /TP B738 /TB SEV"));

        assertFalse(result.getErrors().isEmpty());
        assertFalse(result.isAccepted());
        assertFalse(result.getRejectedWeatherResults().isEmpty());
        assertFalse(result.getPirepResults().get(0).isAccepted());
    }

    @Test
    void usnsRetainsOptionalUnstructuredWeatherWithoutBatchRejection() {
        UsnsIngestResult result = new UsnsIngestService().parse(envelope("CWA ZNY WEATHER ADVISORY HOLD FOR REVIEW"));

        assertTrue(result.getErrors().isEmpty(), result.getErrors().toString());
        assertTrue(result.isAccepted());
        assertEquals(1, result.getClassifiedOnlyWeatherResults().size());
    }

    @Test
    void weatherEngineScenarioFixtureRunsEndToEnd() throws IOException {
        String raw = resource("/scenarios/weather-engine/mixed-usns-weather.txt");
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .rawUsnsMessages(Arrays.asList(raw))
                .route(route())
                .build());

        assertEquals(WeatherDecisionAction.BLOCKED, result.getAction());
        assertFalse(result.getUsnsResults().get(0).getWeatherProducts().isEmpty());
        assertFalse(result.getUsnsResults().get(0).getPirepResults().isEmpty());
        assertTrue(result.getTrace().getSteps().stream().anyMatch(s -> "fuse".equals(s.getStage())));
        assertTrue(result.getTrace().getSteps().stream().anyMatch(s -> "route-impact".equals(s.getStage())));
    }

    @Test
    void realisticMixedScenarioCorpusCoversMultipleWeatherTrafficVariants() throws IOException {
        String raw = resource("/scenarios/weather-engine/mixed-carf-notam-weather-pirep.txt");
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .rawUsnsMessages(Arrays.asList(raw))
                .rawCarfMessages(Arrays.asList(carf()))
                .route(route())
                .notamRestrictions(Arrays.asList(NotamAirspaceRestriction.builder()
                        .id("NOTAM-ROUTE")
                        .effectiveStart(T0.minusMinutes(5))
                        .effectiveEnd(T0.plusHours(4))
                        .lowerAltitudeFeet(24000)
                        .upperAltitudeFeet(26000)
                        .centerLatitude(30)
                        .centerLongitude(-149.5)
                        .radiusNauticalMiles(30)
                        .description("Route restriction")
                        .build()))
                .build());

        assertEquals(WeatherDecisionAction.BLOCKED, result.getAction());
        assertTrue(result.getUsnsResults().get(0).isAccepted());
        assertTrue(result.getTrace().getSteps().stream().anyMatch(step -> !step.getSourceSpans().isEmpty()));
        assertTrue(result.getTrace().getSteps().stream().anyMatch(step -> step.getThresholds().stream()
                .anyMatch(threshold -> threshold.getObservedValue() != null && threshold.getComparator() != null)));
        assertTrue(result.getConstraints().stream().anyMatch(c -> c.getType() == OperationalConstraintType.COMPOUND_OPERATIONAL_RISK));
    }

    @Test
    void expandedWeatherScenarioCorpusCoversForecastSlicesAndLineGeometry() throws IOException {
        WeatherProductParser parser = new WeatherProductParser();
        WeatherProductParseResult tafCwap = parser.parse(resource("/scenarios/weather-engine/taf-cwap-forecast-slices.txt"), null);
        WeatherProductParseResult airmetLine = parser.parse(resource("/scenarios/weather-engine/airmet-sigmet-line-width.txt"), null);
        WeatherProductParseResult metar = parser.parse(resource("/scenarios/weather-engine/metar-low-ceiling-rvr.txt"), null);

        assertEquals(WeatherProductType.TAF, tafCwap.getClassifiedType());
        assertTrue(tafCwap.getProduct().getForecastSlices().size() >= 3);
        assertTrue(tafCwap.getProduct().getForecastSlices().stream().anyMatch(slice -> "TEMPO".equals(slice.getGroupType())));
        assertEquals(WeatherProductType.AIRMET, airmetLine.getClassifiedType());
        assertTrue(airmetLine.isAccepted(), airmetLine.getErrors().toString());
        assertNotNull(airmetLine.getProduct().getLineWidthNauticalMiles());
        assertEquals(WeatherProductType.METAR, metar.getClassifiedType());
        assertEquals(1200.0, metar.getProduct().getRunwayVisualRangeFeet());
    }

    @Test
    void scenarioCorpusCoversStaleWeatherAndRadialPirepRetention() throws IOException {
        String raw = resource("/scenarios/weather-engine/stale-and-radial-pirep.txt");
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .rawUsnsMessages(Arrays.asList(raw))
                .route(route())
                .build());

        assertFalse(result.getUsnsResults().get(0).isAccepted());
        assertTrue(result.getUsnsResults().get(0).getPirepResults().get(0).getReport().getLocationText().contains("JFK"));
        assertTrue(result.getUsnsResults().get(0).getPirepResults().get(0).hasDiagnostic(PirepDiagnosticType.UNRESOLVED_LOCATION_TEXT));
        assertTrue(result.getTrace().getSteps().stream().anyMatch(step -> "classify".equals(step.getStage())
                && !step.getSourceSpans().isEmpty()));
    }

    @Test
    void referenceDataResolvesPirepRadialDmeLocationIntoAcceptedConstraint() {
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .rawUsnsMessages(Arrays.asList(envelope("UUA /TP B738 /OV JFK090020 /TM 0015 /FL250 /TB SEV")))
                .route(route())
                .referenceDataProvider(new InMemoryCarfReferenceDataProvider(Collections.singletonMap("JFK", point(30, -150, 0))))
                .build());

        assertTrue(result.getPirepResults().get(0).isAccepted(), result.getPirepResults().get(0).getDiagnostics().toString());
        assertNotNull(result.getPirepResults().get(0).getReport().getLocation());
        assertTrue(result.getTrace().getSteps().stream().anyMatch(step -> "resolve".equals(step.getStage())));
        assertTrue(result.getConstraints().stream().anyMatch(constraint -> constraint.getType() == OperationalConstraintType.PIREP_HAZARD));
    }

    @Test
    void acceptedModeratePirepBecomesAdvisoryConstraint() {
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .rawUsnsMessages(Arrays.asList(envelope("UA /TP B738 /OV 3000N15000W /TM 0015 /FL250 /TB MOD")))
                .route(route())
                .build());

        OperationalConstraint pirep = result.getConstraints().stream()
                .filter(constraint -> constraint.getType() == OperationalConstraintType.PIREP_HAZARD)
                .findFirst()
                .orElseThrow(AssertionError::new);
        assertEquals(org.tash.extensions.weather.decision.WeatherDecisionSeverity.ADVISORY, pirep.getSeverity());
        assertTrue(pirep.getRationale().contains("Observed PIREP"));
    }

    @Test
    void movingWeatherCanCreateLaterForecastBlockage() {
        WeatherProduct moving = WeatherProduct.builder()
                .id("MOVING")
                .type(WeatherProductType.NEXRAD_POLYGON)
                .source(WeatherProductSource.NOAA)
                .validity(WeatherValidityWindow.builder().validStart(T0).validEnd(T0.plusHours(2)).build())
                .issuedAt(T0)
                .receivedAt(T0)
                .confidence(WeatherConfidence.builder().value(0.9).basis("test").build())
                .provenance("test")
                .movement(WeatherMovementVector.builder().bearingDegrees(180).speedNauticalMilesPerHour(60).build())
                .hazard(CircularWeatherCell.builder()
                        .id("MOVING-H")
                        .type(WeatherElementType.CONVECTION)
                        .severity(HazardSeverity.EXTREME)
                        .startTime(T0)
                        .endTime(T0.plusHours(2))
                        .minAltitude(20000)
                        .maxAltitude(30000)
                        .center(point(31, -149.5, 25000))
                        .radius(12)
                        .build())
                .build();

        RouteWeatherImpactResult result = new WeatherRouteImpactModel().evaluate(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .forecastHorizon(Duration.ofHours(1))
                .forecastStep(Duration.ofHours(1))
                .products(Arrays.asList(moving))
                .build());

        assertFalse(result.getPredictions().get(0).isBlocked());
        assertTrue(result.getPredictions().get(1).isBlocked());
    }

    private WeatherProduct weatherProduct(String id, HazardSeverity severity) {
        return weatherProduct(id, severity, null, 0.0, T0.plusHours(2));
    }

    private WeatherProduct weatherProduct(String id, HazardSeverity severity, Double echoTopFeet,
                                          Double growthTrend, ZonedDateTime endTime) {
        return WeatherProduct.builder()
                .id(id)
                .type(WeatherProductType.NEXRAD_POLYGON)
                .source(WeatherProductSource.NOAA)
                .sourceProduct("NEXRAD")
                .validity(WeatherValidityWindow.builder().validStart(T0).validEnd(endTime).build())
                .issuedAt(T0)
                .receivedAt(T0)
                .confidence(WeatherConfidence.builder().value(0.95).basis("test").build())
                .provenance("test")
                .echoTopFeet(echoTopFeet)
                .growthTrend(growthTrend)
                .hazard(CircularWeatherCell.builder()
                        .id(id + "-H")
                        .type(WeatherElementType.CONVECTION)
                        .severity(severity)
                        .startTime(T0)
                        .endTime(endTime)
                        .minAltitude(20000)
                        .maxAltitude(30000)
                        .center(point(30, -149.5, 25000))
                        .radius(30)
                        .build())
                .build();
    }

    private String carf() {
        return "A. TEST01\n"
                + "B. MISSION\n"
                + "C. LOCATION\n"
                + "D. FL240B260 3000N 15000W 0000 3000N 14900W 0100\n"
                + "F. ETD 200000 MAY 2026 AVANA 200100\n"
                + "G. TAS: 480 KTAS";
    }

    private java.util.List<GeoCoordinate> route() {
        return Arrays.asList(point(30, -150, 25000), point(30, -149, 25000));
    }

    private GeoCoordinate point(double lat, double lon, double alt) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(alt).build();
    }

    private String envelope(String body) {
        return "01GGNC07GP\n"
                + "CNS000 300334\n"
                + "GG KDZZNAXX\n"
                + "300334 KGPS\n"
                + MessageControlCharacters.STX + body
                + MessageControlCharacters.VT + MessageControlCharacters.ETX;
    }

    private String resource(String path) throws IOException {
        try (InputStream stream = OperationalDecisionEngineTest.class.getResourceAsStream(path)) {
            assertNotNull(stream, path);
            return new String(stream.readAllBytes(), StandardCharsets.UTF_8);
        }
    }

    @SuppressWarnings("unchecked")
    private List<OperationalConstraint> invokeCompoundForTest(List<OperationalConstraint> constraints) {
        try {
            java.lang.reflect.Method method = ConstraintFusionService.class.getDeclaredMethod("compoundRisks", List.class);
            method.setAccessible(true);
            return (List<OperationalConstraint>) method.invoke(new ConstraintFusionService(), constraints);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(e);
        }
    }
}
