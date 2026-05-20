package org.tash.extensions.engine;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.carf.api.CarfAnalysisService;
import org.tash.extensions.carf.refdata.CarfReferenceDataProvider;
import org.tash.extensions.messaging.UsnsIngestResult;
import org.tash.extensions.messaging.UsnsIngestService;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.weather.coordination.WeatherCoordinationResult;
import org.tash.extensions.weather.coordination.WeatherCoordinationService;
import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.decision.RouteWeatherDecisionRequest;
import org.tash.extensions.weather.decision.RouteWeatherImpactResult;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;
import org.tash.extensions.weather.decision.WeatherRouteImpactModel;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.pirep.PirepIngestService;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherSourceSpan;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class OperationalDecisionEngine {
    private final UsnsIngestService usnsIngestService;
    private final CarfAnalysisService carfAnalysisService;
    private final PirepIngestService pirepIngestService;
    private final WeatherRouteImpactModel weatherRouteImpactModel;
    private final WeatherCoordinationService weatherCoordinationService;
    private final ConstraintFusionService constraintFusionService;
    private final AuditSigner auditSigner;

    public OperationalDecisionEngine() {
        this(new UsnsIngestService(), new CarfAnalysisService(), new PirepIngestService(),
                new WeatherRouteImpactModel(), new WeatherCoordinationService(), new ConstraintFusionService(),
                new HmacAuditSigner("local-test-key", "airspace-local-audit-secret"));
    }

    public OperationalDecisionEngine(UsnsIngestService usnsIngestService,
                                     CarfAnalysisService carfAnalysisService,
                                     PirepIngestService pirepIngestService,
                                     WeatherRouteImpactModel weatherRouteImpactModel,
                                     WeatherCoordinationService weatherCoordinationService,
                                     ConstraintFusionService constraintFusionService) {
        this(usnsIngestService, carfAnalysisService, pirepIngestService, weatherRouteImpactModel,
                weatherCoordinationService, constraintFusionService,
                new HmacAuditSigner("local-test-key", "airspace-local-audit-secret"));
    }

    public OperationalDecisionEngine(UsnsIngestService usnsIngestService,
                                     CarfAnalysisService carfAnalysisService,
                                     PirepIngestService pirepIngestService,
                                     WeatherRouteImpactModel weatherRouteImpactModel,
                                     WeatherCoordinationService weatherCoordinationService,
                                     ConstraintFusionService constraintFusionService,
                                     AuditSigner auditSigner) {
        this.usnsIngestService = usnsIngestService;
        this.carfAnalysisService = carfAnalysisService;
        this.pirepIngestService = pirepIngestService;
        this.weatherRouteImpactModel = weatherRouteImpactModel;
        this.weatherCoordinationService = weatherCoordinationService;
        this.constraintFusionService = constraintFusionService;
        this.auditSigner = auditSigner == null ? new HmacAuditSigner("local-test-key", "airspace-local-audit-secret") : auditSigner;
    }

    public OperationalDecisionResult evaluate(OperationalDecisionRequest request) {
        OperationalDecisionRequest safe = request == null ? OperationalDecisionRequest.builder().build() : request;
        ZonedDateTime decisionTime = safe.getDecisionTime() == null
                ? ZonedDateTime.parse("2026-05-20T00:00:00Z")
                : safe.getDecisionTime();
        DecisionTrace trace = new DecisionTrace();
        List<UsnsIngestResult> usnsResults = new ArrayList<>();
        List<CarfAnalysisResult> carfResults = new ArrayList<>();
        List<AirspaceReservation> reservations = new ArrayList<>(safe.getReservations());
        List<ReservationConflict> conflicts = new ArrayList<>();
        List<WeatherProduct> weatherProducts = new ArrayList<>(safe.getWeatherProducts());
        List<PirepIngestResult> pirepResults = new ArrayList<>();
        List<NotamAirspaceRestriction> notams = new ArrayList<>(safe.getNotamRestrictions());

        for (String raw : safe.getRawUsnsMessages()) {
            UsnsIngestResult result = usnsIngestService.parse(raw);
            usnsResults.add(result);
            weatherProducts.addAll(result.getWeatherProducts());
            pirepResults.addAll(result.getPirepResults());
            carfResults.addAll(result.getCarfAnalysisResults());
            for (CarfAnalysisResult carf : result.getCarfAnalysisResults()) {
                reservations.addAll(nullSafe(carf.getReservations()));
            }
            trace.add("parse", "Parsed USNS message", decisionTime, source("usns", String.valueOf(usnsResults.size())));
            traceWeatherParsing(trace, result, decisionTime, String.valueOf(usnsResults.size()));
        }

        pirepResults = resolvePirepLocations(pirepResults, safe.getReferenceDataProvider(), decisionTime, trace);

        for (String raw : safe.getRawCarfMessages()) {
            CarfAnalysisResult carf = carfAnalysisService.parseValidateMap(raw);
            carfResults.add(carf);
            reservations.addAll(nullSafe(carf.getReservations()));
            trace.add("map", "Parsed and mapped CARF/ALTRV message", decisionTime, source("carf", String.valueOf(carfResults.size())));
        }

        for (org.tash.extensions.weather.pirep.PirepReport pirep : safe.getPireps()) {
            PirepIngestResult result = pirepIngestService.ingest(pirep, null);
            pirepResults.add(result);
            trace.add("classify", "Ingested PIREP", decisionTime, source("pirep", pirep.getId()));
        }
        pirepResults = resolvePirepLocations(pirepResults, safe.getReferenceDataProvider(), decisionTime, trace);

        CarfAnalysisResult conflictResult = carfAnalysisService.analyzeConflicts(reservations);
        conflicts.addAll(nullSafe(conflictResult.getConflicts()));
        trace.add("conflict", "Evaluated CARF reservation conflicts: " + conflicts.size(), decisionTime,
                source("carfConflict", String.valueOf(conflicts.size())));

        List<GeoCoordinate> route = routeFor(safe, reservations);
        RouteWeatherImpactResult weatherImpact = weatherRouteImpactModel.evaluate(RouteWeatherDecisionRequest.builder()
                .route(route)
                .departureTime(decisionTime)
                .routeDuration(Duration.ofMinutes(60))
                .forecastHorizon(Duration.ofHours(8))
                .forecastStep(Duration.ofHours(1))
                .products(weatherProducts)
                .ensembleProducts(safe.getEnsembleProducts())
                .calibrationModel(safe.getCalibrationModel())
                .sectorDemand(safe.getSectorDemand())
                .routeDemand(safe.getRouteDemand())
                .build());
        List<RouteBlockagePrediction> blockages = weatherImpact.getPredictions();
        trace.add("route-impact", "Evaluated route weather impact across " + blockages.size()
                + " forecast slices", decisionTime, source("weather", String.valueOf(weatherProducts.size())));
        if (blockages.stream().anyMatch(RouteBlockagePrediction::isBlocked)) {
            trace.add("blockage", "Detected blocked route forecast slice", decisionTime,
                    source("weatherBlockage", String.valueOf(blockages.size())));
        }
        traceRouteScoring(trace, blockages, decisionTime);

        WeatherCoordinationResult coordination = weatherCoordinationService.coordinate("operational-route",
                weatherImpact.getAdvisory(), pirepResults);
        if (weatherImpact.getAdvisory() != null && weatherImpact.getAdvisory().getWarnings() != null
                && !weatherImpact.getAdvisory().getWarnings().isEmpty()) {
            trace.addDetailed("warning", "Weather data quality warnings affected the decision", decisionTime,
                    Collections.singletonList(source("weatherWarnings", String.valueOf(weatherImpact.getAdvisory().getWarnings().size()))),
                    Collections.emptyList(), warningIds(blockages, weatherImpact));
        }
        ConstraintFusionResult fusion = constraintFusionService.fuse(ConstraintFusionRequest.builder()
                .reservations(reservations)
                .conflicts(conflicts)
                .notamRestrictions(notams)
                .weatherProducts(weatherProducts)
                .pirepResults(pirepResults)
                .routeBlockages(blockages)
                .build());
        trace.add("fuse", "Fused " + fusion.getConstraints().size() + " operational constraints", decisionTime,
                source("constraints", String.valueOf(fusion.getConstraints().size())));
        ConstraintIndex index = new ConstraintIndex(fusion.getConstraints());
        index.query(RouteCorridor.builder().points(route).bufferNauticalMiles(1.0).build(),
                TimeWindow.builder().start(decisionTime.minusHours(1)).end(decisionTime.plusHours(8)).build(),
                AltitudeBand.builder().lowerFeet(0).upperFeet(60000).build());
        trace.add("index", "Indexed operational constraints", decisionTime, source("constraints", String.valueOf(fusion.getConstraints().size())));

        WeatherDecisionAction action = action(conflicts, blockages, weatherImpact, notams);
        WeatherRecommendedAction recommendedAction = recommendedAction(action, weatherImpact);
        String rationale = rationale(action, conflicts, blockages, notams, weatherImpact);
        double decisionConfidence = Math.min(weatherImpact.getAdvisory().getConfidence(), confidence(blockages));
        trace.addRule("rule-evaluation", "Applied operational action precedence", decisionTime,
                DecisionRuleCatalog.OP_ACTION_PRECEDENCE,
                "decisionConfidence=min(weatherAdvisoryConfidence="
                        + weatherImpact.getAdvisory().getConfidence() + ", blockageConfidence=" + confidence(blockages) + ")",
                Collections.singletonList(DecisionThreshold.builder()
                        .name("route-blocked")
                        .description("Any blocked forecast slice makes route blockage the top action")
                        .comparator(">=")
                        .value(1.0)
                        .observedValue(blockages.stream().anyMatch(RouteBlockagePrediction::isBlocked) ? 1.0 : 0.0)
                        .units("boolean")
                        .build()),
                warningIds(blockages, weatherImpact),
                Collections.singletonList(source("decision", action.name())));
        trace.add("action", "Selected action " + action + ": " + rationale, decisionTime, null);

        OperationalDecisionResult baseResult = OperationalDecisionResult.builder()
                .action(action)
                .recommendedAction(recommendedAction)
                .rationale(rationale)
                .confidence(decisionConfidence)
                .usnsResults(usnsResults)
                .carfAnalysisResults(carfResults)
                .reservations(reservations)
                .conflicts(conflicts)
                .weatherProducts(weatherProducts)
                .pirepResults(pirepResults)
                .routeBlockages(blockages)
                .coordinationResult(coordination)
                .fusionResult(fusion)
                .trace(trace)
                .build();
        OperationalDecisionReplayBundle replayBundle = replayBundle(safe, baseResult, decisionTime);
        OperationalDecisionAuditEnvelope auditEnvelope = auditEnvelope(safe, baseResult, replayBundle, decisionTime);
        replayBundle = replayBundle.toBuilder().auditEnvelope(auditEnvelope).build();
        return baseResult.toBuilder()
                .auditEnvelope(auditEnvelope)
                .replayBundle(replayBundle)
                .build();
    }

    public ReplayVerificationResult replay(OperationalDecisionReplayBundle bundle) {
        List<String> errors = new ArrayList<>();
        List<String> warnings = new ArrayList<>();
        if (bundle == null || bundle.getRequest() == null) {
            errors.add("Replay bundle or request is missing");
            return ReplayVerificationResult.builder().accepted(false).errors(errors).warnings(warnings).build();
        }
        EngineConfig config = bundle.getRequest().getEngineConfig() == null ? EngineConfig.builder().build() : bundle.getRequest().getEngineConfig();
        if (!DecisionRuleCatalog.version().equals(bundle.getRuleCatalogVersion())) {
            String message = "Replay rule catalog version mismatch: expected " + bundle.getRuleCatalogVersion()
                    + " actual " + DecisionRuleCatalog.version();
            if (config.isStrictReplayRuleVersion()) {
                errors.add(message);
            } else {
                warnings.add(message);
            }
        }
        OperationalDecisionResult result = evaluate(bundle.getRequest());
        String resultHash = resultSummaryHash(result);
        if (bundle.getExpectedResultHash() != null && !bundle.getExpectedResultHash().equals(resultHash)) {
            errors.add("Replay result hash mismatch");
        }
        if (bundle.getAuditEnvelope() != null) {
            String payload = bundle.getAuditEnvelope().getRequestHash() + ":" + bundle.getAuditEnvelope().getResultHash()
                    + ":" + bundle.getAuditEnvelope().getRuleCatalogVersion();
            if (!auditSigner.verify(payload, bundle.getAuditEnvelope().getSignature())) {
                errors.add("Replay audit signature verification failed");
            }
        }
        return ReplayVerificationResult.builder()
                .accepted(errors.isEmpty())
                .warnings(warnings)
                .errors(errors)
                .result(result)
                .build();
    }

    private WeatherDecisionAction action(List<ReservationConflict> conflicts, List<RouteBlockagePrediction> blockages,
                                         RouteWeatherImpactResult weatherImpact, List<NotamAirspaceRestriction> notams) {
        if (blockages.stream().anyMatch(RouteBlockagePrediction::isBlocked)) return WeatherDecisionAction.BLOCKED;
        if (!conflicts.isEmpty()) return WeatherDecisionAction.REROUTE;
        if (weatherImpact.getAdvisory().getSeverity() == WeatherDecisionSeverity.WARNING
                || weatherImpact.getAdvisory().getSeverity() == WeatherDecisionSeverity.CRITICAL) {
            return weatherImpact.getAdvisory().getAction();
        }
        if (!notams.isEmpty()) return WeatherDecisionAction.CAUTION;
        return weatherImpact.getAdvisory().getAction();
    }

    private WeatherRecommendedAction recommendedAction(WeatherDecisionAction action, RouteWeatherImpactResult weatherImpact) {
        if (action == WeatherDecisionAction.BLOCKED) return WeatherRecommendedAction.ROUTE_BLOCKED;
        if (action == WeatherDecisionAction.REROUTE && weatherImpact.getAdvisory().getRecommendedAction() != null) {
            return weatherImpact.getAdvisory().getRecommendedAction();
        }
        if (action == WeatherDecisionAction.CAUTION) return WeatherRecommendedAction.MONITOR;
        return weatherImpact.getAdvisory().getRecommendedAction();
    }

    private String rationale(WeatherDecisionAction action, List<ReservationConflict> conflicts,
                             List<RouteBlockagePrediction> blockages, List<NotamAirspaceRestriction> notams,
                             RouteWeatherImpactResult weatherImpact) {
        if (action == WeatherDecisionAction.BLOCKED) return "Route blocked by weather forecast constraints.";
        if (!conflicts.isEmpty()) return "CARF reservation conflicts require operational review or reroute.";
        if (weatherImpact.getAdvisory() != null && weatherImpact.getAdvisory().getRationale() != null) {
            return weatherImpact.getAdvisory().getRationale();
        }
        if (!notams.isEmpty()) return "NOTAM restrictions require caution.";
        return "No blocking operational constraints found.";
    }

    private double confidence(List<RouteBlockagePrediction> blockages) {
        return blockages.stream().mapToDouble(RouteBlockagePrediction::getConfidence).min().orElse(1.0);
    }

    private List<String> warningIds(List<RouteBlockagePrediction> blockages, RouteWeatherImpactResult weatherImpact) {
        List<String> warnings = new ArrayList<>();
        if (blockages != null) {
            for (RouteBlockagePrediction blockage : blockages) {
                if (blockage.isBlocked()) {
                    warnings.add("blocked-forecast-hour-" + blockage.getForecastHour());
                }
            }
        }
        if (weatherImpact != null && weatherImpact.getAdvisory() != null && weatherImpact.getAdvisory().getWarnings() != null) {
            for (int i = 0; i < weatherImpact.getAdvisory().getWarnings().size(); i++) {
                warnings.add("weather-warning-" + i);
            }
        }
        return warnings;
    }

    private void traceRouteScoring(DecisionTrace trace, List<RouteBlockagePrediction> blockages, ZonedDateTime decisionTime) {
        if (trace == null || blockages == null) return;
        for (RouteBlockagePrediction blockage : blockages) {
            for (org.tash.extensions.engine.DecisionRuleApplication application : blockage.getRuleApplications()) {
                DecisionRule rule = application.getRule() == null ? null : DecisionRuleCatalog.byId(application.getRule().getId());
                trace.addRule("route-impact-rule",
                        "Applied route-impact rule " + (rule == null ? "unknown" : rule.getId())
                                + " for forecast hour " + blockage.getForecastHour(),
                        decisionTime,
                        rule,
                        application.getFormula() + "=" + application.getOutputValue(),
                        application.getThresholds(),
                        Collections.emptyList(),
                        Collections.singletonList(source("weatherBlockage", String.valueOf(blockage.getForecastHour()))));
            }
            for (org.tash.extensions.weather.decision.SectorCapacityImpact impact : blockage.getSectorCapacityImpacts()) {
                trace.addRule("capacity-impact",
                        impact.getRationale(),
                        decisionTime,
                        DecisionRuleCatalog.WX_SECTOR_CAPACITY,
                        "capacityImpact=" + impact.getCapacityImpact(),
                        DecisionRuleCatalog.WX_SECTOR_CAPACITY.getDefaultThresholds(),
                        Collections.emptyList(),
                        Collections.singletonList(source("sector", impact.getSectorId())));
            }
        }
    }

    private OperationalDecisionReplayBundle replayBundle(OperationalDecisionRequest request,
                                                         OperationalDecisionResult result,
                                                         ZonedDateTime decisionTime) {
        return OperationalDecisionReplayBundle.builder()
                .bundleVersion("operational-decision-replay-v1")
                .request(request)
                .ruleCatalogVersion(DecisionRuleCatalog.version())
                .engineVersion(engineConfig(request).getEngineVersion())
                .expectedAction(result.getAction() == null ? null : result.getAction().name())
                .expectedRecommendedAction(result.getRecommendedAction() == null ? null : result.getRecommendedAction().name())
                .expectedResultHash(resultSummaryHash(result))
                .decodedProductSummaries(productSummaries(result.getWeatherProducts()))
                .build();
    }

    private OperationalDecisionAuditEnvelope auditEnvelope(OperationalDecisionRequest request,
                                                           OperationalDecisionResult result,
                                                           OperationalDecisionReplayBundle bundle,
                                                           ZonedDateTime decisionTime) {
        EngineConfig config = engineConfig(request);
        String requestHash = CanonicalJson.sha256(requestSummary(request));
        String resultHash = resultSummaryHash(result);
        String configHash = CanonicalJson.sha256(config);
        String payload = requestHash + ":" + resultHash + ":" + DecisionRuleCatalog.version();
        return OperationalDecisionAuditEnvelope.builder()
                .engineVersion(config.getEngineVersion())
                .ruleCatalogVersion(DecisionRuleCatalog.version())
                .requestHash(requestHash)
                .resultHash(resultHash)
                .configHash(configHash)
                .configSnapshot(configSnapshot(config))
                .inputSourceHashes(inputSourceHashes(request))
                .timestamp(decisionTime)
                .signingKeyId(auditSigner.keyId())
                .signature(auditSigner.sign(payload))
                .build();
    }

    private EngineConfig engineConfig(OperationalDecisionRequest request) {
        return request.getEngineConfig() == null ? EngineConfig.builder().build() : request.getEngineConfig();
    }

    private String resultSummaryHash(OperationalDecisionResult result) {
        Map<String, Object> summary = new LinkedHashMap<>();
        summary.put("action", result.getAction() == null ? null : result.getAction().name());
        summary.put("recommendedAction", result.getRecommendedAction() == null ? null : result.getRecommendedAction().name());
        summary.put("confidence", result.getConfidence());
        summary.put("routeBlockageCount", result.getRouteBlockages().size());
        summary.put("constraintCount", result.getConstraints().size());
        return CanonicalJson.sha256(summary);
    }

    private Map<String, Object> requestSummary(OperationalDecisionRequest request) {
        Map<String, Object> summary = new LinkedHashMap<>();
        summary.put("decisionTime", request.getDecisionTime());
        summary.put("rawUsnsMessages", request.getRawUsnsMessages());
        summary.put("rawCarfMessages", request.getRawCarfMessages());
        summary.put("weatherProductIds", request.getWeatherProducts().stream().map(WeatherProduct::getId).toList());
        summary.put("routeSize", request.getRoute().size());
        return summary;
    }

    private List<String> inputSourceHashes(OperationalDecisionRequest request) {
        List<String> hashes = new ArrayList<>();
        request.getRawUsnsMessages().forEach(value -> hashes.add(CanonicalJson.sha256String(value)));
        request.getRawCarfMessages().forEach(value -> hashes.add(CanonicalJson.sha256String(value)));
        request.getWeatherProducts().forEach(value -> hashes.add(CanonicalJson.sha256(value.getId() + ":" + value.getRawText())));
        return hashes;
    }

    private Map<String, Object> configSnapshot(EngineConfig config) {
        Map<String, Object> snapshot = new LinkedHashMap<>();
        snapshot.put("engineVersion", config.getEngineVersion());
        snapshot.put("ruleCatalogVersion", config.getRuleCatalogVersion());
        snapshot.put("indexCellResolutionDegrees", config.getIndexCellResolutionDegrees());
        snapshot.put("timeBucketMinutes", config.getTimeBucketMinutes());
        snapshot.put("altitudeBucketFeet", config.getAltitudeBucketFeet());
        return snapshot;
    }

    private List<Map<String, Object>> productSummaries(List<WeatherProduct> products) {
        List<Map<String, Object>> summaries = new ArrayList<>();
        for (WeatherProduct product : nullSafe(products)) {
            Map<String, Object> summary = new LinkedHashMap<>();
            summary.put("id", product.getId());
            summary.put("type", product.getType() == null ? null : product.getType().name());
            summary.put("sourceProduct", product.getSourceProduct());
            summary.put("confidence", product.confidenceValue());
            summary.put("forecastHour", product.getForecastHour());
            summaries.add(summary);
        }
        return summaries;
    }

    private List<GeoCoordinate> routeFor(OperationalDecisionRequest request, List<AirspaceReservation> reservations) {
        if (request.getRoute() != null && request.getRoute().size() >= 2) return request.getRoute();
        for (AirspaceReservation reservation : reservations) {
            if (reservation.getRouteStart() != null && reservation.getRouteEnd() != null) {
                List<GeoCoordinate> route = new ArrayList<>();
                route.add(reservation.getRouteStart());
                route.add(reservation.getRouteEnd());
                return route;
            }
        }
        return Collections.emptyList();
    }

    private <T> List<T> nullSafe(List<T> values) {
        return values == null ? Collections.emptyList() : values;
    }

    private DecisionSourceRef source(String type, String id) {
        return DecisionSourceRef.builder().type(type).id(id).description(type + ":" + id).build();
    }

    private List<PirepIngestResult> resolvePirepLocations(List<PirepIngestResult> results,
                                                          CarfReferenceDataProvider referenceDataProvider,
                                                          ZonedDateTime decisionTime,
                                                          DecisionTrace trace) {
        if (referenceDataProvider == null || results == null || results.isEmpty()) {
            return results;
        }
        List<PirepIngestResult> resolved = new ArrayList<>();
        for (PirepIngestResult result : results) {
            if (result == null || result.getReport() == null || result.getReport().getLocation() != null) {
                resolved.add(result);
                continue;
            }
            Optional<GeoCoordinate> coordinate = resolvePirepLocation(result.getReport().getLocationText(), referenceDataProvider);
            if (!coordinate.isPresent()) {
                resolved.add(result);
                continue;
            }
            org.tash.extensions.weather.pirep.PirepReport report = result.getReport().toBuilder()
                    .location(coordinate.get())
                    .locationQuality(0.75)
                    .build();
            PirepIngestResult reingested = pirepIngestService.ingest(report, null);
            resolved.add(reingested);
            trace.add("resolve", "Resolved PIREP location text " + result.getReport().getLocationText(),
                    decisionTime, source("pirepLocation", result.getReport().getLocationText()));
        }
        return resolved;
    }

    private Optional<GeoCoordinate> resolvePirepLocation(String locationText,
                                                        CarfReferenceDataProvider referenceDataProvider) {
        if (locationText == null || locationText.trim().isEmpty()) {
            return Optional.empty();
        }
        String normalized = locationText.trim().toUpperCase(java.util.Locale.US);
        Matcher radialDme = Pattern.compile("^([A-Z0-9]{2,5})(\\d{3})(\\d{3})$").matcher(normalized);
        if (radialDme.find()) {
            Optional<GeoCoordinate> base = referenceDataProvider.resolveFixOrNavaid(radialDme.group(1));
            if (base.isPresent()) {
                double bearing = Double.parseDouble(radialDme.group(2));
                double distance = Double.parseDouble(radialDme.group(3));
                return Optional.of(base.get().destinationPoint(distance, bearing));
            }
        }
        return referenceDataProvider.resolveFixOrNavaid(normalized);
    }

    private void traceWeatherParsing(DecisionTrace trace, UsnsIngestResult result, ZonedDateTime decisionTime, String sourceId) {
        for (WeatherProductParseResult weather : result.getWeatherProductResults()) {
            List<DecisionSourceSpan> spans = new ArrayList<>();
            for (WeatherSourceSpan span : weather.getSourceSpans()) {
                spans.add(DecisionSourceSpan.builder()
                        .sourceId(sourceId)
                        .field(span.getField())
                        .startOffset(span.getStartOffset())
                        .endOffset(span.getEndOffset())
                        .text(span.getText())
                        .build());
            }
            trace.addDetailed("classify", "Classified weather transaction as " + weather.getClassifiedType(),
                    decisionTime, Collections.singletonList(source("weatherProduct", weather.getClassifiedType().name())),
                    spans, weather.getErrors());
        }
    }
}
