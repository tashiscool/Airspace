package org.tash.extensions.engine;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;
import org.tash.extensions.weather.avoid.CircularWeatherCell;
import org.tash.extensions.weather.decision.AirspaceSector;
import org.tash.extensions.weather.decision.RouteImpactCalibrationModel;
import org.tash.extensions.weather.decision.RouteWeatherDecisionRequest;
import org.tash.extensions.weather.decision.RouteWeatherImpactResult;
import org.tash.extensions.weather.decision.SectorCapacityImpact;
import org.tash.extensions.weather.decision.SectorDemandSnapshot;
import org.tash.extensions.weather.decision.StormCellLifecycle;
import org.tash.extensions.weather.decision.StormCellLifecycleTracker;
import org.tash.extensions.weather.decision.WeatherRouteImpactModel;
import org.tash.extensions.weather.product.EnsembleWeatherProduct;
import org.tash.extensions.weather.product.WeatherConfidence;
import org.tash.extensions.weather.product.WeatherEnsembleMember;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductSource;
import org.tash.extensions.weather.product.WeatherProductType;
import org.tash.extensions.weather.product.WeatherValidityWindow;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class ProductionHardeningTest {
    private static final ZonedDateTime T0 = ZonedDateTime.parse("2026-05-20T00:00:00Z");

    @Test
    void routeBlockagePredictionsCarryCatalogLinkedRuleApplicationsAndBreakdown() {
        RouteWeatherImpactResult result = new WeatherRouteImpactModel().evaluate(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .forecastHorizon(Duration.ofHours(1))
                .forecastStep(Duration.ofHours(1))
                .products(Collections.singletonList(weather("WX", HazardSeverity.EXTREME, 0.95)))
                .build());

        assertFalse(result.getPredictions().isEmpty());
        assertNotNull(result.getPredictions().get(0).getScoringBreakdown());
        assertFalse(result.getPredictions().get(0).getRuleApplications().isEmpty());
        assertTrue(result.getPredictions().get(0).getRuleApplications().stream()
                .allMatch(application -> DecisionRuleCatalog.byId(application.getRule().getId()) != null));
        assertFalse(result.getPredictions().get(0).getScoringBreakdown().getRuleApplications().isEmpty());
    }

    @Test
    void auditEnvelopeAndReplayBundleAreDeterministicAndTamperAware() {
        OperationalDecisionRequest request = OperationalDecisionRequest.builder()
                .decisionTime(T0)
                .route(route())
                .weatherProducts(Collections.singletonList(weather("WX", HazardSeverity.EXTREME, 0.95)))
                .build();
        OperationalDecisionEngine engine = new OperationalDecisionEngine();
        OperationalDecisionResult first = engine.evaluate(request);
        OperationalDecisionResult second = engine.evaluate(request);

        assertNotNull(first.getAuditEnvelope());
        assertNotNull(first.getReplayBundle());
        assertEquals(first.getAuditEnvelope().getRequestHash(), second.getAuditEnvelope().getRequestHash());
        assertEquals(first.getAuditEnvelope().getResultHash(), second.getAuditEnvelope().getResultHash());
        assertTrue(engine.replay(first.getReplayBundle()).isAccepted());

        OperationalDecisionReplayBundle tampered = first.getReplayBundle().toBuilder()
                .expectedResultHash("tampered")
                .build();
        assertFalse(engine.replay(tampered).isAccepted());
    }

    @Test
    void strongerSpatialIndexFiltersByRouteTimeAltitudeAndValidity() {
        OperationalConstraint onRoute = constraint("on", T0, T0.plusHours(1), 20000, 30000,
                Arrays.asList(point(30, -150), point(30, -149)));
        OperationalConstraint far = constraint("far", T0, T0.plusHours(1), 20000, 30000,
                Arrays.asList(point(40, -120), point(41, -120)));
        OperationalConstraint expired = constraint("expired", T0.minusHours(3), T0.minusHours(2), 20000, 30000,
                Arrays.asList(point(30, -150), point(30, -149)));
        ConstraintSpatialIndex index = new ConstraintSpatialIndex(Arrays.asList(onRoute, far, expired),
                EngineConfig.builder().indexCellResolutionDegrees(0.25).build());

        List<OperationalConstraint> matches = index.query(RouteCorridor.builder().points(route()).bufferNauticalMiles(10).build(),
                TimeWindow.builder().start(T0).end(T0.plusHours(2)).build(),
                AltitudeBand.builder().lowerFeet(24000).upperFeet(26000).build(),
                ProductValidityPolicy.CURRENT_ONLY);

        assertEquals(Collections.singletonList("on"), matches.stream().map(OperationalConstraint::getId).toList());
        assertTrue(index.indexCellCount() > 0);
        assertTrue(index.candidateCount(RouteCorridor.builder().points(route()).bufferNauticalMiles(10).build()) < 3);
    }

    @Test
    void syntheticNasScaleIndexMatchesBruteForceWithFewerCandidates() {
        List<OperationalConstraint> constraints = new java.util.ArrayList<>();
        for (int i = 0; i < 2_000; i++) {
            double lat = 20.0 + (i % 100) * 0.2;
            double lon = -130.0 + (i / 100) * 0.2;
            constraints.add(constraint("far-" + i, T0, T0.plusHours(4), 1000, 45000,
                    Arrays.asList(point(lat, lon), point(lat + 0.02, lon + 0.02))));
        }
        OperationalConstraint onRoute = constraint("documented-route-weather", T0.plusMinutes(15), T0.plusHours(1),
                23000, 27000, Arrays.asList(point(30, -150.05), point(30.05, -149.95)));
        OperationalConstraint wrongAltitude = constraint("wrong-altitude", T0.plusMinutes(15), T0.plusHours(1),
                1000, 5000, Arrays.asList(point(30, -150.05), point(30.05, -149.95)));
        OperationalConstraint expired = constraint("expired-near-route", T0.minusHours(3), T0.minusHours(2),
                23000, 27000, Arrays.asList(point(30, -150.05), point(30.05, -149.95)));
        constraints.add(onRoute);
        constraints.add(wrongAltitude);
        constraints.add(expired);

        RouteCorridor corridor = RouteCorridor.builder().points(route()).bufferNauticalMiles(20).build();
        TimeWindow window = TimeWindow.builder().start(T0).end(T0.plusHours(2)).build();
        AltitudeBand band = AltitudeBand.builder().lowerFeet(24000).upperFeet(26000).build();
        ConstraintSpatialIndex index = new ConstraintSpatialIndex(constraints,
                EngineConfig.builder().indexCellResolutionDegrees(0.10).build());

        List<String> indexed = index.query(corridor, window, band, ProductValidityPolicy.CURRENT_ONLY)
                .stream().map(OperationalConstraint::getId).sorted().toList();
        List<String> bruteForce = constraints.stream()
                .filter(c -> !c.getEndTime().isBefore(window.getStart()) && !c.getStartTime().isAfter(window.getEnd()))
                .filter(c -> c.getUpperAltitudeFeet() >= band.getLowerFeet() && c.getLowerAltitudeFeet() <= band.getUpperFeet())
                .filter(c -> c.getGeometry().stream().mapToDouble(GeoCoordinate::getLatitude).max().orElse(0) >= corridor.minLatitude())
                .filter(c -> c.getGeometry().stream().mapToDouble(GeoCoordinate::getLatitude).min().orElse(0) <= corridor.maxLatitude())
                .filter(c -> c.getGeometry().stream().mapToDouble(GeoCoordinate::getLongitude).max().orElse(0) >= corridor.minLongitude())
                .filter(c -> c.getGeometry().stream().mapToDouble(GeoCoordinate::getLongitude).min().orElse(0) <= corridor.maxLongitude())
                .map(OperationalConstraint::getId)
                .sorted()
                .toList();

        assertEquals(bruteForce, indexed);
        assertEquals(Collections.singletonList("documented-route-weather"), indexed);
        assertTrue(index.candidateCount(corridor) < constraints.size() / 5,
                "Index should prefilter documented synthetic NAS-scale corpus before geometry checks");
    }

    @Test
    void calibrationEnsembleLifecycleAndCapacityAffectRouteImpactDeterministically() {
        WeatherProduct highConfidence = weather("ENS-HI", HazardSeverity.SEVERE, 0.95);
        WeatherProduct lowConfidence = weather("ENS-LO", HazardSeverity.SEVERE, 0.45);
        EnsembleWeatherProduct ensemble = EnsembleWeatherProduct.builder()
                .id("ENS")
                .members(Arrays.asList(
                        WeatherEnsembleMember.builder().id("m1").weight(0.5).product(highConfidence).build(),
                        WeatherEnsembleMember.builder().id("m2").weight(0.5).product(lowConfidence).build()))
                .build();
        AirspaceSector sector = AirspaceSector.builder()
                .id("ZNY-TEST")
                .baselineCapacityPerHour(20)
                .boundary(Arrays.asList(point(29.5, -150.5), point(29.5, -148.5), point(30.5, -148.5), point(30.5, -150.5)))
                .build();
        RouteImpactCalibrationModel calibration = RouteImpactCalibrationModel.builder()
                .probabilityScale(0.80)
                .build();

        RouteWeatherImpactResult result = new WeatherRouteImpactModel().evaluate(RouteWeatherDecisionRequest.builder()
                .route(route())
                .departureTime(T0)
                .forecastHorizon(Duration.ofHours(1))
                .forecastStep(Duration.ofHours(1))
                .products(Collections.singletonList(weather("WX", HazardSeverity.SEVERE, 0.80)))
                .ensembleProducts(Collections.singletonList(ensemble))
                .sectorDemand(Collections.singletonList(SectorDemandSnapshot.builder()
                        .sector(sector)
                        .validTime(T0)
                        .activeDemandPerHour(30)
                        .build()))
                .calibrationModel(calibration)
                .build());

        assertTrue(result.getPredictions().get(0).getEnsembleSpread() > 0.0);
        assertTrue(result.getPredictions().get(0).getConfidenceIntervalHigh()
                > result.getPredictions().get(0).getConfidenceIntervalLow());
        assertFalse(result.getPredictions().get(0).getSectorCapacityImpacts().isEmpty());
        SectorCapacityImpact impact = result.getPredictions().get(0).getSectorCapacityImpacts().get(0);
        assertEquals("ZNY-TEST", impact.getSectorId());

        List<StormCellLifecycle> lifecycles = new StormCellLifecycleTracker().track(Arrays.asList(
                weather("CELL-1", HazardSeverity.SEVERE, 0.8).toBuilder().growthTrend(1.0).build(),
                weather("CELL-2", HazardSeverity.SEVERE, 0.8).toBuilder().growthTrend(-1.0).build()));
        assertEquals(1, lifecycles.size());
        assertEquals(2, lifecycles.get(0).getProducts().size());
    }

    private OperationalConstraint constraint(String id, ZonedDateTime start, ZonedDateTime end,
                                             double lower, double upper, List<GeoCoordinate> geometry) {
        return OperationalConstraint.builder()
                .id(id)
                .type(OperationalConstraintType.WEATHER_HAZARD)
                .startTime(start)
                .endTime(end)
                .lowerAltitudeFeet(lower)
                .upperAltitudeFeet(upper)
                .geometry(geometry)
                .build();
    }

    private WeatherProduct weather(String id, HazardSeverity severity, double confidence) {
        return WeatherProduct.builder()
                .id(id)
                .type(WeatherProductType.NEXRAD_POLYGON)
                .source(WeatherProductSource.NOAA)
                .sourceProduct("CWAP")
                .validity(WeatherValidityWindow.builder().validStart(T0).validEnd(T0.plusHours(2)).build())
                .issuedAt(T0)
                .receivedAt(T0)
                .confidence(WeatherConfidence.builder().value(confidence).basis("test").build())
                .provenance("test")
                .echoTopFeet(42000.0)
                .growthTrend(1.0)
                .hazard(CircularWeatherCell.builder()
                        .id(id + "-cell")
                        .type(WeatherElementType.CONVECTION)
                        .severity(severity)
                        .startTime(T0)
                        .endTime(T0.plusHours(2))
                        .minAltitude(20000)
                        .maxAltitude(30000)
                        .center(point(30, -149.5))
                        .radius(30)
                        .build())
                .build();
    }

    private List<GeoCoordinate> route() {
        return Arrays.asList(point(30, -150), point(30, -149));
    }

    private GeoCoordinate point(double lat, double lon) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(25000).build();
    }
}
