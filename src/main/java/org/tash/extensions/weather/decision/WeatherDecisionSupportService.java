package org.tash.extensions.weather.decision;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.HazardousWeather;
import org.tash.extensions.engine.DecisionRuleApplication;
import org.tash.extensions.engine.DecisionRuleCatalog;
import org.tash.extensions.engine.DecisionRuleRef;
import org.tash.extensions.engine.DecisionThreshold;
import org.tash.extensions.weather.avoid.SimpleDeviationStrategy;
import org.tash.extensions.weather.avoid.WeatherAvoidanceStrategy;
import org.tash.extensions.weather.avoid.WeatherCell;
import org.tash.extensions.weather.product.WeatherDiagnosticType;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductDiagnostic;
import org.tash.extensions.weather.product.WeatherProductType;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class WeatherDecisionSupportService {
    private static final Duration DEFAULT_STALE_AFTER = Duration.ofMinutes(30);

    private final WeatherAvoidanceStrategy avoidanceStrategy;
    private final Duration staleAfter;
    private final CapacityImpactModel capacityImpactModel = new CapacityImpactModel();

    public WeatherDecisionSupportService() {
        this(new SimpleDeviationStrategy(), DEFAULT_STALE_AFTER);
    }

    public WeatherDecisionSupportService(WeatherAvoidanceStrategy avoidanceStrategy, Duration staleAfter) {
        this.avoidanceStrategy = avoidanceStrategy == null ? new SimpleDeviationStrategy() : avoidanceStrategy;
        this.staleAfter = staleAfter == null ? DEFAULT_STALE_AFTER : staleAfter;
    }

    public RouteWeatherAdvisory adviseRoute(List<GeoCoordinate> route,
                                            ZonedDateTime departureTime,
                                            Duration routeDuration,
                                            List<WeatherHazardSnapshot> hazards) {
        return adviseRoute(RouteWeatherDecisionRequest.builder()
                .route(route == null ? Collections.emptyList() : route)
                .departureTime(departureTime)
                .routeDuration(routeDuration)
                .hazards(hazards == null ? Collections.emptyList() : hazards)
                .build());
    }

    public RouteWeatherAdvisory adviseRoute(RouteWeatherDecisionRequest request) {
        if (request == null) {
            return RouteWeatherAdvisory.builder()
                    .action(WeatherDecisionAction.CLEAR)
                    .recommendedAction(WeatherRecommendedAction.NONE)
                    .severity(WeatherDecisionSeverity.INFO)
                    .rationale("No weather decision request was provided.")
                    .build();
        }
        ZonedDateTime departureTime = request.getDepartureTime();
        List<WeatherHazardSnapshot> hazards = snapshotsFor(request, departureTime);
        List<RouteHazardIntersection> intersections = intersections(request.route(), departureTime,
                request.routeDurationOrDefault(), hazards);
        List<String> warnings = productWarnings(hazards, departureTime);
        warnings.addAll(productDiagnosticWarnings(request.products(), departureTime));
        WeatherDecisionSeverity severity = severity(intersections, warnings);
        WeatherDecisionAction action = action(intersections, warnings);
        WeatherRecommendedAction recommendedAction = recommendedAction(action, intersections);
        List<ImpactedRouteSegment> impactedSegments = impactedSegments(request.route(), intersections);
        List<GeoCoordinate> suggestedPath = suggestedPath(request.route(), departureTime, hazards, action);
        List<RouteBlockagePrediction> blockagePredictions = predictRouteBlockage(request, false);

        return RouteWeatherAdvisory.builder()
                .action(action)
                .recommendedAction(recommendedAction)
                .severity(severity)
                .rationale(rationale(action, intersections, warnings))
                .confidence(confidence(intersections))
                .primaryHazardId(primaryHazardId(intersections))
                .intersections(intersections)
                .impactedSegments(impactedSegments)
                .warnings(warnings)
                .suggestedPath(suggestedPath)
                .blockagePredictions(blockagePredictions)
                .build();
    }

    public List<RouteBlockagePrediction> predictRouteBlockage(List<GeoCoordinate> route,
                                                              ZonedDateTime startTime,
                                                              Duration horizon,
                                                              Duration step,
                                                              Duration routeDuration,
                                                              List<WeatherHazardSnapshot> hazards) {
        return predictRouteBlockage(RouteWeatherDecisionRequest.builder()
                .route(route == null ? Collections.emptyList() : route)
                .departureTime(startTime)
                .forecastHorizon(horizon)
                .forecastStep(step)
                .routeDuration(routeDuration)
                .hazards(hazards == null ? Collections.emptyList() : hazards)
                .build());
    }

    public List<RouteBlockagePrediction> predictRouteBlockage(RouteWeatherDecisionRequest request) {
        return predictRouteBlockage(request, true);
    }

    private List<RouteBlockagePrediction> predictRouteBlockage(RouteWeatherDecisionRequest request, boolean includeNestedAdvisory) {
        if (request == null || request.route().size() < 2 || request.getDepartureTime() == null) {
            return Collections.emptyList();
        }
        Duration horizon = request.forecastHorizonOrDefault();
        Duration step = request.forecastStepOrDefault();
        List<RouteBlockagePrediction> predictions = new ArrayList<>();
        ZonedDateTime cursor = request.getDepartureTime();
        ZonedDateTime end = request.getDepartureTime().plus(horizon);
        while (!cursor.isAfter(end)) {
            List<WeatherHazardSnapshot> hazards = snapshotsFor(request, cursor);
            List<RouteHazardIntersection> intersections = intersections(request.route(), cursor,
                    request.routeDurationOrDefault(), hazards);
            List<String> warnings = productWarnings(hazards, cursor);
            warnings.addAll(productDiagnosticWarnings(request.products(), cursor));
            WeatherDecisionAction action = action(intersections, warnings);
            WeatherDecisionSeverity severity = severity(intersections, warnings);
            WeatherRecommendedAction recommendedAction = recommendedAction(action, intersections);
            int forecastHour = (int) Duration.between(request.getDepartureTime(), cursor).toHours();
            RouteImpactCalibrationModel calibration = request.calibrationOrDefault();
            RouteImpactScoringBreakdown breakdown = scoringBreakdown(intersections, forecastHour, calibration);
            double blockedProbability = breakdown.getBlockedProbability();
            boolean blocked = action == WeatherDecisionAction.BLOCKED || blockedProbability >= calibration.getBlockedThreshold();
            RouteBlockagePrediction partial = RouteBlockagePrediction.builder()
                    .forecastTime(cursor)
                    .forecastHour(forecastHour)
                    .blocked(blocked)
                    .severity(severity)
                    .rationale(rationale(action, intersections, warnings))
                    .recommendedAction(recommendedAction)
                    .primaryHazardId(primaryHazardId(intersections))
                    .confidence(leadTimeConfidence(confidence(intersections), forecastHour, calibration))
                    .deviationLikelihood(breakdown.getDeviationLikelihood())
                    .capacityImpact(breakdown.getCapacityImpact())
                    .blockedProbability(blockedProbability)
                    .confidenceMath("confidence=minIntersectionConfidence(" + confidence(intersections)
                            + ") * leadTimeCurve(hour=" + forecastHour + ")")
                    .ruleIds(ruleIds(intersections, blockedProbability, calibration))
                    .ruleApplications(breakdown.getRuleApplications())
                    .scoringBreakdown(breakdown)
                    .ensembleSpread(ensembleSpread(request, cursor))
                    .confidenceIntervalLow(clamp(blockedProbability - ensembleSpread(request, cursor)))
                    .confidenceIntervalHigh(clamp(blockedProbability + ensembleSpread(request, cursor)))
                    .blockedSegmentIndexes(blockedSegmentIndexes(intersections, calibration))
                    .segmentBlockedProbabilities(segmentBlockedProbabilities(intersections, forecastHour, calibration))
                    .intersections(intersections)
                    .build();
            List<SectorCapacityImpact> sectorImpacts = capacityImpactModel.impacts(partial, request.sectorDemand(), partial.getConfidence());
            double capacityImpact = Math.max(partial.getCapacityImpact(), sectorImpacts.stream()
                    .mapToDouble(SectorCapacityImpact::getCapacityImpact)
                    .max()
                    .orElse(0.0));
            predictions.add(partial.toBuilder()
                    .capacityImpact(capacityImpact)
                    .sectorCapacityImpacts(sectorImpacts)
                    .build());
            cursor = cursor.plus(step);
        }
        return predictions;
    }

    public List<RouteHazardIntersection> intersections(List<GeoCoordinate> route,
                                                       ZonedDateTime departureTime,
                                                       Duration routeDuration,
                                                       List<WeatherHazardSnapshot> hazards) {
        if (route == null || route.size() < 2 || departureTime == null || hazards == null || hazards.isEmpty()) {
            return Collections.emptyList();
        }
        Duration duration = routeDuration == null || routeDuration.isNegative() || routeDuration.isZero()
                ? Duration.ofMinutes(route.size() - 1)
                : routeDuration;
        List<RouteHazardIntersection> result = new ArrayList<>();
        int segments = route.size() - 1;
        for (int i = 0; i < segments; i++) {
            GeoCoordinate start = route.get(i);
            GeoCoordinate end = route.get(i + 1);
            ZonedDateTime segmentTime = departureTime.plus(duration.multipliedBy(i).dividedBy(segments));
            for (WeatherHazardSnapshot snapshot : hazards) {
                if (snapshot == null || snapshot.getHazard() == null) {
                    continue;
                }
                HazardousWeather hazard = snapshot.getHazard();
                if (intersects(hazard, start, end, segmentTime)) {
                    result.add(RouteHazardIntersection.builder()
                            .hazardId(hazard.getId())
                            .productId(snapshot.getProductId())
                            .hazardType(hazard.getType())
                            .hazardSeverity(hazard.getSeverity())
                            .segmentIndex(i)
                            .estimatedEntryTime(segmentTime)
                            .estimatedExitTime(segmentTime.plus(duration.dividedBy(Math.max(1, segments))))
                            .estimatedEntryPoint(start)
                            .estimatedExitPoint(end)
                            .confidence(snapshot.getConfidence())
                            .productStatus(snapshot.statusAt(departureTime, staleAfter))
                            .sourceProduct(snapshot.getSourceProduct())
                            .provenance(snapshot.getProvenance())
                            .forecastHour(snapshot.getForecastHour())
                            .productType(snapshot.getProductType())
                            .movement(snapshot.getMovement())
                            .echoTopFeet(snapshot.getEchoTopFeet())
                            .growthTrend(snapshot.getGrowthTrend())
                            .stormPhase(snapshot.getStormPhase())
                            .recommendedAction(recommendedActionForHazard(hazard.getSeverity(), snapshot.getProductType()))
                            .rationale("Route segment intersects " + hazard.getSeverity() + " " + hazard.getType())
                            .build());
                }
            }
        }
        return result;
    }

    private boolean intersects(HazardousWeather hazard, GeoCoordinate start, GeoCoordinate end, ZonedDateTime time) {
        if (hazard instanceof WeatherCell) {
            WeatherCell cell = (WeatherCell) hazard;
            if (!altitudeOverlaps(start, end, cell)) {
                return false;
            }
            return cell.intersectsPath(start, end, time);
        }
        return hazard.affectsPoint(start, time) || hazard.affectsPoint(end, time);
    }

    private boolean altitudeOverlaps(GeoCoordinate start, GeoCoordinate end, WeatherCell cell) {
        double routeMin = Math.min(start.getAltitude(), end.getAltitude());
        double routeMax = Math.max(start.getAltitude(), end.getAltitude());
        return routeMax >= cell.getMinAltitude() && routeMin <= cell.getMaxAltitude();
    }

    private List<String> productWarnings(List<WeatherHazardSnapshot> hazards, ZonedDateTime decisionTime) {
        List<String> warnings = new ArrayList<>();
        if (hazards == null) {
            return warnings;
        }
        for (WeatherHazardSnapshot hazard : hazards) {
            if (hazard == null) {
                continue;
            }
            WeatherProductStatus status = hazard.statusAt(decisionTime, staleAfter);
            if (status == WeatherProductStatus.STALE) {
                warnings.add("Weather product " + hazard.getProductId() + " is stale");
            } else if (status == WeatherProductStatus.FUTURE) {
                warnings.add("Weather product " + hazard.getProductId() + " is issued after decision time");
            } else if (status == WeatherProductStatus.LOW_CONFIDENCE) {
                warnings.add("Weather product " + hazard.getProductId() + " has low confidence");
            }
        }
        return warnings;
    }

    private List<String> productDiagnosticWarnings(List<WeatherProduct> products, ZonedDateTime decisionTime) {
        List<String> warnings = new ArrayList<>();
        if (products == null) {
            return warnings;
        }
        for (WeatherProduct product : products) {
            if (product == null) {
                continue;
            }
            for (WeatherProductDiagnostic diagnostic : product.diagnosticsAt(decisionTime, staleAfter)) {
                if (diagnostic.getType() == WeatherDiagnosticType.STALE
                        || diagnostic.getType() == WeatherDiagnosticType.LOW_CONFIDENCE
                        || diagnostic.getType() == WeatherDiagnosticType.EXPIRED
                        || diagnostic.getType() == WeatherDiagnosticType.FUTURE_ISSUED) {
                    warnings.add(diagnostic.getMessage());
                }
            }
        }
        return warnings;
    }

    private WeatherDecisionSeverity severity(List<RouteHazardIntersection> intersections, List<String> warnings) {
        if (intersections.stream().anyMatch(i -> i.getHazardSeverity() == HazardSeverity.EXTREME)) {
            return WeatherDecisionSeverity.CRITICAL;
        }
        if (intersections.stream().anyMatch(i -> i.getHazardSeverity() == HazardSeverity.SEVERE)) {
            return WeatherDecisionSeverity.WARNING;
        }
        if (!intersections.isEmpty() || !warnings.isEmpty()) {
            return WeatherDecisionSeverity.ADVISORY;
        }
        return WeatherDecisionSeverity.INFO;
    }

    private WeatherDecisionAction action(List<RouteHazardIntersection> intersections, List<String> warnings) {
        if (intersections.stream().anyMatch(i -> i.getHazardSeverity() == HazardSeverity.EXTREME)) {
            return WeatherDecisionAction.BLOCKED;
        }
        if (intersections.stream().anyMatch(i -> i.getProductType() == WeatherProductType.CEILING
                || i.getProductType() == WeatherProductType.VISIBILITY)) {
            return WeatherDecisionAction.DELAY;
        }
        if (intersections.stream().anyMatch(i -> i.getProductType() == WeatherProductType.ICING
                || i.getProductType() == WeatherProductType.TURBULENCE)) {
            return WeatherDecisionAction.ALTITUDE_CHANGE;
        }
        if (intersections.stream().anyMatch(i -> i.getHazardSeverity() == HazardSeverity.SEVERE)) {
            return WeatherDecisionAction.REROUTE;
        }
        if (!intersections.isEmpty()) {
            return WeatherDecisionAction.CAUTION;
        }
        if (!warnings.isEmpty()) {
            return WeatherDecisionAction.MONITOR;
        }
        return WeatherDecisionAction.CLEAR;
    }

    private WeatherRecommendedAction recommendedAction(WeatherDecisionAction action,
                                                       List<RouteHazardIntersection> intersections) {
        if (action == WeatherDecisionAction.BLOCKED) {
            return WeatherRecommendedAction.ROUTE_BLOCKED;
        }
        if (action == WeatherDecisionAction.REROUTE) {
            return WeatherRecommendedAction.REROUTE_AROUND_WEATHER;
        }
        if (action == WeatherDecisionAction.ALTITUDE_CHANGE) {
            return WeatherRecommendedAction.CLIMB_OR_DESCEND;
        }
        if (action == WeatherDecisionAction.DELAY) {
            return WeatherRecommendedAction.DELAY_DEPARTURE;
        }
        if (action == WeatherDecisionAction.CAUTION) {
            return intersections.isEmpty() ? WeatherRecommendedAction.MONITOR : WeatherRecommendedAction.AVOID_CELL;
        }
        if (action == WeatherDecisionAction.MONITOR) {
            return WeatherRecommendedAction.HOLD_FOR_UPDATE;
        }
        return WeatherRecommendedAction.NONE;
    }

    private WeatherRecommendedAction recommendedActionForHazard(HazardSeverity severity, WeatherProductType productType) {
        if (severity == HazardSeverity.EXTREME) {
            return WeatherRecommendedAction.ROUTE_BLOCKED;
        }
        if (productType == WeatherProductType.CEILING || productType == WeatherProductType.VISIBILITY) {
            return WeatherRecommendedAction.DELAY_DEPARTURE;
        }
        if (productType == WeatherProductType.ICING || productType == WeatherProductType.TURBULENCE) {
            return WeatherRecommendedAction.CLIMB_OR_DESCEND;
        }
        if (severity == HazardSeverity.SEVERE) {
            return WeatherRecommendedAction.REROUTE_AROUND_WEATHER;
        }
        return WeatherRecommendedAction.AVOID_CELL;
    }

    private String rationale(WeatherDecisionAction action,
                             List<RouteHazardIntersection> intersections,
                             List<String> warnings) {
        if (intersections.isEmpty() && warnings.isEmpty()) {
            return "No active weather hazards intersect the planned route.";
        }
        return "Action " + action + " from " + intersections.size()
                + " route/weather intersections and " + warnings.size() + " product warnings.";
    }

    private List<GeoCoordinate> suggestedPath(List<GeoCoordinate> route,
                                              ZonedDateTime departureTime,
                                              List<WeatherHazardSnapshot> hazards,
                                              WeatherDecisionAction action) {
        if (route == null || route.size() < 2 || hazards == null || action == WeatherDecisionAction.CLEAR) {
            return route == null ? Collections.emptyList() : new ArrayList<>(route);
        }
        List<HazardousWeather> rawHazards = new ArrayList<>();
        for (WeatherHazardSnapshot hazard : hazards) {
            if (hazard != null && hazard.getHazard() != null) {
                rawHazards.add(hazard.getHazard());
            }
        }
        return avoidanceStrategy.findAvoidancePath(route.get(0), route.get(route.size() - 1), departureTime, rawHazards);
    }

    private List<WeatherHazardSnapshot> snapshotsFor(RouteWeatherDecisionRequest request, ZonedDateTime decisionTime) {
        List<WeatherHazardSnapshot> snapshots = new ArrayList<>();
        if (request.getProducts() != null) {
            for (WeatherProduct product : request.getProducts()) {
                if (product != null) {
                    for (WeatherProduct expanded : product.expandForecastSlices()) {
                        if (isSliceApplicable(expanded, decisionTime)) {
                            snapshots.add(expanded.toSnapshotAt(decisionTime));
                        }
                    }
                }
            }
        }
        if (request.getEnsembleProducts() != null) {
            request.getEnsembleProducts().forEach(ensemble -> {
                if (ensemble != null) {
                    for (WeatherProduct product : ensemble.products()) {
                        for (WeatherProduct expanded : product.expandForecastSlices()) {
                            if (isSliceApplicable(expanded, decisionTime)) {
                                snapshots.add(expanded.toSnapshotAt(decisionTime));
                            }
                        }
                    }
                }
            });
        }
        if (request.getHazards() != null) {
            snapshots.addAll(request.getHazards());
        }
        return snapshots;
    }

    private boolean isSliceApplicable(WeatherProduct product, ZonedDateTime decisionTime) {
        if (product == null || product.getValidity() == null || decisionTime == null) {
            return true;
        }
        ZonedDateTime start = product.getValidity().getValidStart();
        ZonedDateTime end = product.getValidity().getValidEnd();
        if (start != null && decisionTime.isBefore(start)) {
            return false;
        }
        return end == null || !decisionTime.isAfter(end);
    }

    private List<ImpactedRouteSegment> impactedSegments(List<GeoCoordinate> route,
                                                        List<RouteHazardIntersection> intersections) {
        List<ImpactedRouteSegment> impacted = new ArrayList<>();
        if (intersections == null) {
            return impacted;
        }
        for (RouteHazardIntersection intersection : intersections) {
            GeoCoordinate start = route != null && intersection.getSegmentIndex() < route.size()
                    ? route.get(intersection.getSegmentIndex())
                    : intersection.getEstimatedEntryPoint();
            GeoCoordinate end = route != null && intersection.getSegmentIndex() + 1 < route.size()
                    ? route.get(intersection.getSegmentIndex() + 1)
                    : intersection.getEstimatedExitPoint();
            impacted.add(ImpactedRouteSegment.builder()
                    .segmentIndex(intersection.getSegmentIndex())
                    .segmentStart(start)
                    .segmentEnd(end)
                    .estimatedEntryTime(intersection.getEstimatedEntryTime())
                    .estimatedExitTime(intersection.getEstimatedExitTime())
                    .primaryHazardId(intersection.getHazardId())
                    .primaryHazardType(intersection.getHazardType())
                    .primaryHazardSeverity(intersection.getHazardSeverity())
                    .confidence(intersection.getConfidence())
                    .recommendedAction(intersection.getRecommendedAction())
                    .rationale(intersection.getRationale())
                    .build());
        }
        return impacted;
    }

    private double confidence(List<RouteHazardIntersection> intersections) {
        if (intersections == null || intersections.isEmpty()) {
            return 1.0;
        }
        return intersections.stream()
                .map(RouteHazardIntersection::getConfidence)
                .min(Comparator.naturalOrder())
                .orElse(0.0);
    }

    private String primaryHazardId(List<RouteHazardIntersection> intersections) {
        if (intersections == null || intersections.isEmpty()) {
            return null;
        }
        return intersections.stream()
                .sorted(Comparator.comparing(RouteHazardIntersection::getHazardSeverity).reversed())
                .map(RouteHazardIntersection::getHazardId)
                .findFirst()
                .orElse(null);
    }

    private List<Integer> blockedSegmentIndexes(List<RouteHazardIntersection> intersections, RouteImpactCalibrationModel calibration) {
        Set<Integer> indexes = intersections == null ? new LinkedHashSet<>() : intersections.stream()
                .filter(i -> i.getHazardSeverity() == HazardSeverity.EXTREME || score(i, i.getForecastHour() == null ? 0 : i.getForecastHour(), calibration) >= calibration.getBlockedThreshold())
                .map(RouteHazardIntersection::getSegmentIndex)
                .collect(Collectors.toCollection(LinkedHashSet::new));
        return new ArrayList<>(indexes);
    }

    private Map<Integer, Double> segmentBlockedProbabilities(List<RouteHazardIntersection> intersections, int forecastHour, RouteImpactCalibrationModel calibration) {
        Map<Integer, Double> probabilities = new LinkedHashMap<>();
        if (intersections == null) {
            return probabilities;
        }
        for (RouteHazardIntersection intersection : intersections) {
            double probability = score(intersection, forecastHour, calibration) * calibration.getProbabilityScale();
            probabilities.merge(intersection.getSegmentIndex(), probability, Math::max);
        }
        return probabilities;
    }

    private double blockedProbability(List<RouteHazardIntersection> intersections, int forecastHour, RouteImpactCalibrationModel calibration) {
        return intersections == null ? 0.0 : intersections.stream()
                .mapToDouble(i -> score(i, forecastHour, calibration) * calibration.getProbabilityScale())
                .max()
                .orElse(0.0);
    }

    private List<String> ruleIds(List<RouteHazardIntersection> intersections, double blockedProbability, RouteImpactCalibrationModel calibration) {
        List<String> rules = new ArrayList<>();
        if (blockedProbability >= calibration.getBlockedThreshold()) {
            rules.add(DecisionRuleCatalog.WX_BLOCKED_PROBABILITY.getId());
        }
        if (intersections != null) {
            for (RouteHazardIntersection intersection : intersections) {
                if (intersection.getEchoTopFeet() != null) {
                    rules.add(DecisionRuleCatalog.WX_ECHO_TOPS_ROUTE_ALTITUDE.getId());
                    break;
                }
            }
            for (RouteHazardIntersection intersection : intersections) {
                if (intersection.getGrowthTrend() != null && intersection.getGrowthTrend() != 0.0) {
                    rules.add(DecisionRuleCatalog.WX_GROWTH_DECAY_TREND.getId());
                    break;
                }
            }
        }
        return rules;
    }

    private double deviationLikelihood(List<RouteHazardIntersection> intersections, int forecastHour, RouteImpactCalibrationModel calibration) {
        return intersections == null ? 0.0 : intersections.stream()
                .mapToDouble(i -> score(i, forecastHour, calibration))
                .max()
                .orElse(0.0);
    }

    private double capacityImpact(List<RouteHazardIntersection> intersections, int forecastHour, RouteImpactCalibrationModel calibration) {
        double max = intersections == null ? 0.0 : intersections.stream()
                .mapToDouble(i -> score(i, forecastHour, calibration) * (i.getHazardSeverity() == HazardSeverity.EXTREME ? 1.0 : 0.75))
                .max()
                .orElse(0.0);
        return clamp(max + Math.min(calibration.getMaxCapacityBoost(), (intersections == null ? 0 : intersections.size()) * calibration.getMultiIntersectionCapacityBoost()));
    }

    private double score(RouteHazardIntersection intersection, int forecastHour, RouteImpactCalibrationModel calibration) {
        double growth = intersection.getGrowthTrend() == null ? 0.0 : intersection.getGrowthTrend() * calibration.getGrowthMultiplier();
        double phase = stormPhaseWeight(intersection.getStormPhase());
        return clamp((calibration.severityWeight(intersection.getHazardSeverity()) + growth)
                * leadTimeConfidence(intersection.getConfidence(), forecastHour, calibration)
                * echoTopFactor(intersection)
                * phase
                * calibration.productWeight(intersection.getProductType()));
    }

    private double echoTopFactor(RouteHazardIntersection intersection) {
        if (intersection.getEchoTopFeet() == null || intersection.getEstimatedEntryPoint() == null) {
            return 1.0;
        }
        return intersection.getEchoTopFeet() + 1000.0 < intersection.getEstimatedEntryPoint().getAltitude() ? 0.2 : 1.0;
    }

    private double stormPhaseWeight(String phase) {
        if ("GROWING".equals(phase) || "DEVELOPING".equals(phase)) return 1.1;
        if ("MATURE".equals(phase)) return 1.0;
        if ("DECAYING".equals(phase)) return 0.75;
        return 1.0;
    }

    private double leadTimeConfidence(double confidence, int forecastHour, RouteImpactCalibrationModel calibration) {
        return clamp(confidence * Math.max(calibration.getMinimumLeadTimeConfidence(), 1.0 - Math.max(0, forecastHour) * calibration.getLeadTimeDecayPerHour()));
    }

    private RouteImpactScoringBreakdown scoringBreakdown(List<RouteHazardIntersection> intersections, int forecastHour, RouteImpactCalibrationModel calibration) {
        RouteHazardIntersection primary = intersections == null || intersections.isEmpty() ? null : intersections.get(0);
        double severity = primary == null ? 0.0 : calibration.severityWeight(primary.getHazardSeverity());
        double growth = primary == null || primary.getGrowthTrend() == null ? 0.0 : primary.getGrowthTrend() * calibration.getGrowthMultiplier();
        double phase = primary == null ? 1.0 : stormPhaseWeight(primary.getStormPhase());
        double echo = primary == null ? 1.0 : echoTopFactor(primary);
        double lead = primary == null ? 1.0 : leadTimeConfidence(primary.getConfidence(), forecastHour, calibration);
        double deviation = deviationLikelihood(intersections, forecastHour, calibration);
        double capacity = capacityImpact(intersections, forecastHour, calibration);
        double probability = blockedProbability(intersections, forecastHour, calibration);
        List<DecisionRuleApplication> applications = new ArrayList<>();
        applications.add(application(DecisionRuleCatalog.WX_SEVERITY_WEIGHT, severity, severity, "severityWeight"));
        applications.add(application(DecisionRuleCatalog.WX_LEAD_TIME_CONFIDENCE, (double) forecastHour, lead, "leadTimeConfidence"));
        applications.add(application(DecisionRuleCatalog.WX_ECHO_TOPS_ROUTE_ALTITUDE, primary == null || primary.getEchoTopFeet() == null ? null : primary.getEchoTopFeet(), echo, "echoTopFactor"));
        applications.add(application(DecisionRuleCatalog.WX_GROWTH_DECAY_TREND, primary == null ? null : primary.getGrowthTrend(), growth, "growthAdjustment"));
        applications.add(application(DecisionRuleCatalog.WX_BLOCKED_PROBABILITY, probability, probability, "blockedProbability"));
        applications.add(application(DecisionRuleCatalog.WX_SECTOR_CAPACITY, capacity, capacity, "capacityImpact"));
        return RouteImpactScoringBreakdown.builder()
                .severityWeight(severity)
                .growthAdjustment(growth)
                .stormPhaseWeight(phase)
                .echoTopFactor(echo)
                .leadTimeConfidence(lead)
                .calibratedScore(deviation)
                .blockedProbability(probability)
                .deviationLikelihood(deviation)
                .capacityImpact(capacity)
                .ruleApplications(applications)
                .build();
    }

    private DecisionRuleApplication application(org.tash.extensions.engine.DecisionRule rule, Double observed, Double output, String formula) {
        return DecisionRuleApplication.builder()
                .rule(DecisionRuleRef.of(rule))
                .input(rule == null ? null : rule.getDescription())
                .formula(formula)
                .observedValue(observed)
                .outputValue(output)
                .rationale(rule == null ? null : rule.getDefaultRationaleTemplate())
                .thresholds(rule == null ? Collections.emptyList() : rule.getDefaultThresholds())
                .build();
    }

    private double ensembleSpread(RouteWeatherDecisionRequest request, ZonedDateTime cursor) {
        if (request == null || request.getEnsembleProducts() == null || request.getEnsembleProducts().isEmpty()) {
            return 0.0;
        }
        List<Double> confidences = new ArrayList<>();
        request.getEnsembleProducts().forEach(ensemble -> ensemble.getMembers().forEach(member -> {
            if (member != null && member.getProduct() != null) {
                confidences.add(member.getProduct().confidenceValue());
            }
        }));
        if (confidences.size() < 2) {
            return 0.0;
        }
        double mean = confidences.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        double variance = confidences.stream().mapToDouble(value -> Math.pow(value - mean, 2)).average().orElse(0.0);
        return Math.min(0.5, Math.sqrt(variance));
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
