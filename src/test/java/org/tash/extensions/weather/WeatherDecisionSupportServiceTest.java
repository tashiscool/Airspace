package org.tash.extensions.weather;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.avoid.CircularWeatherCell;
import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.decision.RouteWeatherDecisionRequest;
import org.tash.extensions.weather.decision.RouteWeatherAdvisory;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;
import org.tash.extensions.weather.decision.WeatherDecisionSupportService;
import org.tash.extensions.weather.decision.WeatherHazardSnapshot;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;
import org.tash.extensions.weather.coordination.MeteorologistReviewPriority;
import org.tash.extensions.weather.coordination.WeatherCoordinationResult;
import org.tash.extensions.weather.coordination.WeatherCoordinationService;
import org.tash.extensions.weather.pirep.AutomatedPirepDraft;
import org.tash.extensions.weather.pirep.PirepIngestService;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.pirep.PirepIntensity;
import org.tash.extensions.weather.pirep.PirepDiagnosticType;
import org.tash.extensions.weather.pirep.ListPirepRepositoryView;
import org.tash.extensions.weather.pirep.PirepPhenomenon;
import org.tash.extensions.weather.pirep.PirepReport;
import org.tash.extensions.weather.pirep.PirepValidationResult;
import org.tash.extensions.weather.product.WeatherConfidence;
import org.tash.extensions.weather.product.WeatherDiagnosticType;
import org.tash.extensions.weather.product.WeatherMovementVector;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductSource;
import org.tash.extensions.weather.product.WeatherProductType;
import org.tash.extensions.weather.product.WeatherValidityWindow;
import org.tash.extensions.visualization.AirspaceFeature;
import org.tash.extensions.visualization.AirspaceFeatureCollection;
import org.tash.extensions.visualization.AirspaceVisualizationService;
import org.tash.extensions.visualization.VisualizationRequest;

import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class WeatherDecisionSupportServiceTest {
    private static final ZonedDateTime T0 = ZonedDateTime.parse("2026-05-20T12:00:00Z");

    @Test
    void severeConvectionIntersectingRouteProducesRerouteAdvisory() {
        WeatherHazardSnapshot convection = snapshot(cell("WX1", WeatherElementType.CONVECTION, HazardSeverity.SEVERE,
                T0.minusMinutes(5), T0.plusHours(1), point(30, -149.5, 25000), 30), 0.9);

        RouteWeatherAdvisory advisory = new WeatherDecisionSupportService().adviseRoute(route(),
                T0, Duration.ofMinutes(45), Arrays.asList(convection));

        assertEquals(WeatherDecisionAction.REROUTE, advisory.getAction());
        assertEquals(WeatherDecisionSeverity.WARNING, advisory.getSeverity());
        assertEquals(WeatherRecommendedAction.REROUTE_AROUND_WEATHER, advisory.getRecommendedAction());
        assertEquals(1, advisory.getIntersections().size());
        assertEquals(1, advisory.getImpactedSegments().size());
        assertEquals("WX1", advisory.getIntersections().get(0).getHazardId());
        assertFalse(advisory.getSuggestedPath().isEmpty());
    }

    @Test
    void extremeHazardPredictsRouteBlockageAcrossForecastSlices() {
        WeatherHazardSnapshot convectiveLine = snapshot(cell("LINE", WeatherElementType.CONVECTION, HazardSeverity.EXTREME,
                T0, T0.plusHours(8), point(30, -149.5, 25000), 25), 0.95);

        List<RouteBlockagePrediction> predictions = new WeatherDecisionSupportService().predictRouteBlockage(route(),
                T0, Duration.ofHours(2), Duration.ofHours(1), Duration.ofMinutes(50), Arrays.asList(convectiveLine));

        assertEquals(3, predictions.size());
        assertTrue(predictions.stream().allMatch(RouteBlockagePrediction::isBlocked));
        assertTrue(predictions.stream().allMatch(p -> p.getSeverity() == WeatherDecisionSeverity.CRITICAL));
        assertEquals(WeatherRecommendedAction.ROUTE_BLOCKED, predictions.get(0).getRecommendedAction());
        assertFalse(predictions.get(0).getBlockedSegmentIndexes().isEmpty());
    }

    @Test
    void structuredWeatherProductsPreserveValidityConfidenceMovementAndDiagnostics() {
        WeatherProduct product = WeatherProduct.builder()
                .id("SIGMET-1")
                .type(WeatherProductType.SIGMET)
                .source(WeatherProductSource.FAA)
                .sourceProduct("SIGMET")
                .provider("WTIC")
                .validity(WeatherValidityWindow.builder().validStart(T0).validEnd(T0.plusHours(2)).build())
                .issuedAt(T0.minusMinutes(20))
                .receivedAt(T0.minusMinutes(15))
                .forecastHour(1)
                .confidence(WeatherConfidence.builder().value(0.91).basis("model+forecaster").build())
                .provenance("structured-test")
                .movement(WeatherMovementVector.builder().speedNauticalMilesPerHour(35).bearingDegrees(90).build())
                .hazard(cell("SIG-H", WeatherElementType.CONVECTION, HazardSeverity.SEVERE,
                        T0, T0.plusHours(2), point(30, -149.5, 25000), 25))
                .build();

        assertEquals(300, product.latency().getSeconds());
        assertTrue(product.diagnosticsAt(T0, Duration.ofMinutes(30)).isEmpty());
        assertEquals("SIGMET-1", product.toSnapshotAt(T0).getProductId());

        WeatherProduct staleLowConfidence = WeatherProduct.builder()
                .id("TAF-LOW")
                .type(WeatherProductType.TAF)
                .source(WeatherProductSource.NWS)
                .validity(WeatherValidityWindow.builder().validStart(T0.minusHours(3)).validEnd(T0.minusHours(1)).build())
                .issuedAt(T0.minusHours(3))
                .confidence(WeatherConfidence.builder().value(0.25).basis("old forecast").build())
                .build();

        assertTrue(staleLowConfidence.diagnosticsAt(T0, Duration.ofMinutes(30)).stream()
                .anyMatch(d -> d.getType() == WeatherDiagnosticType.EXPIRED));
        assertTrue(staleLowConfidence.diagnosticsAt(T0, Duration.ofMinutes(30)).stream()
                .anyMatch(d -> d.getType() == WeatherDiagnosticType.LOW_CONFIDENCE));
    }

    @Test
    void altitudeSeparatedIcingDoesNotIntersectRoute() {
        WeatherHazardSnapshot icing = snapshot(cell("ICE", WeatherElementType.ICING, HazardSeverity.SEVERE,
                T0.minusMinutes(5), T0.plusHours(1), point(30, -149.5, 12000), 30,
                10000, 14000), 0.8);

        RouteWeatherAdvisory advisory = new WeatherDecisionSupportService().adviseRoute(route(),
                T0, Duration.ofMinutes(45), Arrays.asList(icing));

        assertEquals(WeatherDecisionAction.CLEAR, advisory.getAction());
        assertTrue(advisory.getIntersections().isEmpty());
    }

    @Test
    void ceilingOrVisibilityProductRecommendsDelayInsteadOfReroute() {
        WeatherProduct ceiling = WeatherProduct.builder()
                .id("METAR-LIFR")
                .type(WeatherProductType.CEILING)
                .source(WeatherProductSource.NOAA)
                .sourceProduct("METAR")
                .validity(WeatherValidityWindow.builder().validStart(T0.minusMinutes(5)).validEnd(T0.plusMinutes(55)).build())
                .issuedAt(T0.minusMinutes(5))
                .receivedAt(T0.minusMinutes(4))
                .confidence(WeatherConfidence.builder().value(0.95).basis("observation").build())
                .provenance("metar-test")
                .hazard(cell("CEIL", WeatherElementType.CEILING, HazardSeverity.SEVERE,
                        T0.minusMinutes(5), T0.plusMinutes(55), point(30, -149.5, 25000), 20))
                .build();

        RouteWeatherAdvisory advisory = new WeatherDecisionSupportService().adviseRoute(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .routeDuration(Duration.ofMinutes(45))
                .products(Arrays.asList(ceiling))
                .build());

        assertEquals(WeatherDecisionAction.DELAY, advisory.getAction());
        assertEquals(WeatherRecommendedAction.DELAY_DEPARTURE, advisory.getRecommendedAction());
        assertEquals(WeatherProductType.CEILING, advisory.getIntersections().get(0).getProductType());
    }

    @Test
    void movingWeatherVectorShiftsForecastBlockageOverTime() {
        WeatherProduct moving = WeatherProduct.builder()
                .id("NEXRAD-MOVING")
                .type(WeatherProductType.NEXRAD_POLYGON)
                .source(WeatherProductSource.NOAA)
                .sourceProduct("NEXRAD")
                .validity(WeatherValidityWindow.builder().validStart(T0).validEnd(T0.plusHours(2)).build())
                .issuedAt(T0.minusMinutes(2))
                .receivedAt(T0.minusMinutes(1))
                .confidence(WeatherConfidence.builder().value(0.9).basis("radar extrapolation").build())
                .provenance("movement-test")
                .movement(WeatherMovementVector.builder().speedNauticalMilesPerHour(60).bearingDegrees(180).build())
                .hazard(cell("MOV", WeatherElementType.CONVECTION, HazardSeverity.EXTREME,
                        T0, T0.plusHours(2), point(31, -149.5, 25000), 12))
                .build();

        List<RouteBlockagePrediction> predictions = new WeatherDecisionSupportService().predictRouteBlockage(
                RouteWeatherDecisionRequest.builder()
                        .route(route())
                        .departureTime(T0)
                        .forecastHorizon(Duration.ofHours(1))
                        .forecastStep(Duration.ofHours(1))
                        .routeDuration(Duration.ofMinutes(30))
                        .products(Arrays.asList(moving))
                        .build());

        assertFalse(predictions.get(0).isBlocked());
        assertTrue(predictions.get(1).isBlocked());
        assertEquals(1, predictions.get(1).getForecastHour());
    }

    @Test
    void staleWeatherProductCausesMonitorWarningEvenWhenRouteIsClear() {
        WeatherHazardSnapshot stale = snapshot(cell("OLD", WeatherElementType.TURBULENCE, HazardSeverity.MODERATE,
                T0.minusHours(3), T0.plusHours(1), point(35, -140, 25000), 10), 0.8,
                T0.minusHours(2));

        RouteWeatherAdvisory advisory = new WeatherDecisionSupportService().adviseRoute(route(),
                T0, Duration.ofMinutes(45), Arrays.asList(stale));

        assertEquals(WeatherDecisionAction.MONITOR, advisory.getAction());
        assertFalse(advisory.getWarnings().isEmpty());
        assertTrue(advisory.getWarnings().get(0).contains("stale"));
    }

    @Test
    void pirepValidationRequiresAutomationCapturedCoreFields() {
        Clock clock = Clock.fixed(Instant.parse("2026-05-20T12:05:00Z"), ZoneOffset.UTC);
        PirepIngestService service = new PirepIngestService(clock, Duration.ofHours(2));
        AutomatedPirepDraft draft = service.automatedDraft("AAL123", "B738", T0, point(30, -149.5, 25000));

        PirepReport report = draft.toReport(PirepPhenomenon.TURBULENCE, PirepIntensity.SEVERE, "SEV TURB");
        PirepValidationResult accepted = service.validate(report);
        PirepValidationResult rejected = service.validate(PirepReport.builder()
                .observationTime(T0)
                .phenomenon(PirepPhenomenon.ICING)
                .build());

        assertTrue(accepted.isAccepted());
        assertTrue(report.isUrgent());
        assertFalse(rejected.isAccepted());
        assertTrue(rejected.getErrors().contains("Aircraft type is required"));
        assertTrue(rejected.getErrors().contains("Location is required"));
        assertTrue(rejected.getErrors().contains("Altitude is required"));
        assertTrue(rejected.hasDiagnostic(PirepDiagnosticType.MISSING_LOCATION));
    }

    @Test
    void pirepIngestDetectsDuplicateAndModelsDisseminationWithoutPersistence() {
        Clock clock = Clock.fixed(Instant.parse("2026-05-20T12:05:00Z"), ZoneOffset.UTC);
        PirepIngestService service = new PirepIngestService(clock, Duration.ofHours(2));
        PirepReport existing = service.automatedDraft("AAL123", "B738", T0, point(30, -149.5, 25000))
                .toReport(PirepPhenomenon.TURBULENCE, PirepIntensity.MODERATE, "MOD TURB");
        PirepReport duplicate = service.automatedDraft("AAL123", "B738", T0.plusMinutes(4), point(30.02, -149.48, 25000))
                .toReport(PirepPhenomenon.TURBULENCE, PirepIntensity.MODERATE, "MOD TURB AGAIN");

        PirepIngestResult result = service.ingest(duplicate, new ListPirepRepositoryView(Arrays.asList(existing)));

        assertFalse(result.isAccepted());
        assertTrue(result.hasDiagnostic(PirepDiagnosticType.DUPLICATE));
    }

    @Test
    void urgentPirepAndBlockedRouteCreateCoordinationReviewItemsAndConstraints() {
        WeatherHazardSnapshot convectiveLine = snapshot(cell("LINE", WeatherElementType.CONVECTION, HazardSeverity.EXTREME,
                T0, T0.plusHours(1), point(30, -149.5, 25000), 25), 0.95);
        RouteWeatherAdvisory advisory = new WeatherDecisionSupportService().adviseRoute(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .forecastHorizon(Duration.ofHours(1))
                .forecastStep(Duration.ofHours(1))
                .hazards(Arrays.asList(convectiveLine))
                .build());
        PirepReport urgent = PirepReport.builder()
                .id("UA1")
                .aircraftType("B738")
                .observationTime(T0)
                .location(point(30, -149.5, 25000))
                .altitudeFeet(25000.0)
                .phenomenon(PirepPhenomenon.TURBULENCE)
                .intensity(PirepIntensity.SEVERE)
                .urgent(true)
                .build();
        PirepIngestResult pirepResult = new PirepIngestService(Clock.fixed(Instant.parse("2026-05-20T12:05:00Z"), ZoneOffset.UTC),
                Duration.ofHours(2)).ingest(urgent, new ListPirepRepositoryView(Arrays.asList()));

        WeatherCoordinationResult result = new WeatherCoordinationService(Clock.fixed(Instant.parse("2026-05-20T12:06:00Z"), ZoneOffset.UTC))
                .coordinate("RTE1", advisory, Arrays.asList(pirepResult));

        assertEquals(MeteorologistReviewPriority.URGENT, result.getAdvisory().getReviewPriority());
        assertFalse(result.getReviewItems().isEmpty());
        assertFalse(result.getConstraints().isEmpty());
        assertEquals(WeatherRecommendedAction.ROUTE_BLOCKED, result.getConstraints().get(0).getRecommendedAction());
        assertEquals("RTE1", result.getHandoffNotes().get(0).getRouteId());
    }

    @Test
    void weatherHazardsAndAdvisoriesExportAsEngineGeoJsonFeatures() {
        WeatherHazardSnapshot convection = snapshot(cell("WX1", WeatherElementType.CONVECTION, HazardSeverity.SEVERE,
                T0.minusMinutes(5), T0.plusHours(1), point(30, -149.5, 25000), 30), 0.9);
        AirspaceVisualizationService visualization = new AirspaceVisualizationService();
        AirspaceFeatureCollection weatherFeatures = visualization.featuresForWeather(Arrays.asList(convection));
        RouteWeatherAdvisory advisory = new WeatherDecisionSupportService().adviseRoute(route(),
                T0, Duration.ofMinutes(45), Arrays.asList(convection));
        AirspaceFeatureCollection advisoryFeatures = visualization.featuresForWeatherAdvisory(advisory);

        assertEquals("FeatureCollection", weatherFeatures.getType());
        assertEquals("Polygon", weatherFeatures.getFeatures().get(0).getGeometry().getType());
        assertEquals("weather", weatherFeatures.getFeatures().get(0).getProperties().get("featureKind"));
        AirspaceFeature intersection = advisoryFeatures.getFeatures().get(0);
        assertEquals("weather-intersection", intersection.getProperties().get("featureKind"));
        assertEquals(WeatherElementType.CONVECTION, intersection.getProperties().get("hazardType"));
    }

    @Test
    void weatherProductsAndAdvisoriesExportComprehensiveFeatureProperties() {
        WeatherProduct product = WeatherProduct.builder()
                .id("NEXRAD-1")
                .type(WeatherProductType.NEXRAD_POLYGON)
                .source(WeatherProductSource.NOAA)
                .sourceProduct("NEXRAD")
                .provider("radar")
                .validity(WeatherValidityWindow.builder().validStart(T0).validEnd(T0.plusHours(1)).build())
                .issuedAt(T0.minusMinutes(5))
                .receivedAt(T0.minusMinutes(4))
                .forecastHour(0)
                .confidence(WeatherConfidence.builder().value(0.88).basis("radar").build())
                .movement(WeatherMovementVector.builder().speedNauticalMilesPerHour(30).bearingDegrees(80).build())
                .provenance("visual-test")
                .hazard(cell("NEX-H", WeatherElementType.CONVECTION, HazardSeverity.SEVERE,
                        T0, T0.plusHours(1), point(30, -149.5, 25000), 20))
                .build();
        RouteWeatherAdvisory advisory = new WeatherDecisionSupportService().adviseRoute(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .products(Arrays.asList(product))
                .build());
        VisualizationRequest request = new VisualizationRequest();
        request.setWeatherProducts(Arrays.asList(product));
        request.setWeatherAdvisories(Arrays.asList(advisory));

        AirspaceFeatureCollection collection = new AirspaceVisualizationService().combined(request);

        assertTrue(collection.getFeatures().stream()
                .anyMatch(f -> "weather-product".equals(f.getProperties().get("featureKind"))
                        && WeatherProductType.NEXRAD_POLYGON.equals(f.getProperties().get("productType"))
                        && f.getProperties().containsKey("movementBearingDegrees")));
        assertTrue(collection.getFeatures().stream()
                .anyMatch(f -> "weather-intersection".equals(f.getProperties().get("featureKind"))
                        && f.getProperties().containsKey("recommendedAction")));
    }

    private List<GeoCoordinate> route() {
        return Arrays.asList(point(30, -150, 25000), point(30, -149, 25000));
    }

    private CircularWeatherCell cell(String id, WeatherElementType type, HazardSeverity severity,
                                     ZonedDateTime start, ZonedDateTime end, GeoCoordinate center, double radius) {
        return cell(id, type, severity, start, end, center, radius, 20000, 30000);
    }

    private CircularWeatherCell cell(String id, WeatherElementType type, HazardSeverity severity,
                                     ZonedDateTime start, ZonedDateTime end, GeoCoordinate center, double radius,
                                     double minAltitude, double maxAltitude) {
        return CircularWeatherCell.builder()
                .id(id)
                .type(type)
                .severity(severity)
                .startTime(start)
                .endTime(end)
                .minAltitude(minAltitude)
                .maxAltitude(maxAltitude)
                .center(center)
                .radius(radius)
                .build();
    }

    private WeatherHazardSnapshot snapshot(HazardousWeather hazard, double confidence) {
        return snapshot(hazard, confidence, T0.minusMinutes(5));
    }

    private WeatherHazardSnapshot snapshot(HazardousWeather hazard, double confidence, ZonedDateTime issuedAt) {
        return WeatherHazardSnapshot.builder()
                .productId("P-" + hazard.getId())
                .sourceProduct("synthetic-nextgen-weather")
                .provider("test")
                .hazard(hazard)
                .issuedAt(issuedAt)
                .receivedAt(issuedAt.plusMinutes(1))
                .confidence(confidence)
                .provenance("unit-test")
                .build();
    }

    private GeoCoordinate point(double lat, double lon, double alt) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(alt).build();
    }
}
