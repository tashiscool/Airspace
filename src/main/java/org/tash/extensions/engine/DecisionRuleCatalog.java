package org.tash.extensions.engine;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public final class DecisionRuleCatalog {
    private static final String VERSION = "weather-engine-rules-2026-05-20";
    public static final DecisionRule OP_ACTION_PRECEDENCE = rule(
            "OP-ACTION-PRECEDENCE-001",
            "action-precedence",
            "warning",
            "Select the top operational action in precedence order: route blocked, unresolved CARF conflict, severe weather, NOTAM restriction, monitor, clear.",
            "Selected {action} because {dominantConstraint} outranks lower-severity constraints.",
            DecisionThreshold.builder().name("route-blocked").description("Any blocked forecast slice selects BLOCKED").comparator(">=").value(1.0).units("boolean").build());
    public static final DecisionRule WX_BLOCKED_PROBABILITY = rule(
            "WX-BLOCKED-PROBABILITY-GTE-0.80",
            "route-impact",
            "warning",
            "Treat a route forecast slice as blocked when its weather impact score reaches or exceeds 0.80.",
            "Blocked probability {blockedProbability} reached the route blockage threshold.",
            DecisionThreshold.builder().name("blocked-probability").description("Forecast slice blocked probability").comparator(">=").value(0.80).units("probability").build());
    public static final DecisionRule WX_ECHO_TOPS_ROUTE_ALTITUDE = rule(
            "WX-ECHO-TOPS-VS-ROUTE-ALTITUDE",
            "route-impact",
            "info",
            "Reduce convective impact when echo tops are materially below the route altitude band.",
            "Echo tops {echoTopFeet} compared to route altitude {routeAltitudeFeet}.",
            DecisionThreshold.builder().name("echo-top-margin").description("Echo top altitude must be within 1000 feet of route altitude to retain full impact").comparator(">=").value(-1000.0).units("feet").build());
    public static final DecisionRule WX_GROWTH_DECAY_TREND = rule(
            "WX-GROWTH-DECAY-TREND",
            "route-impact",
            "info",
            "Increase or decrease impact according to parsed growth, decay, and storm-phase indicators.",
            "Storm trend {trend} adjusted route impact.",
            DecisionThreshold.builder().name("growth-trend").description("Positive growth increases impact; negative decay reduces it").comparator("!=").value(0.0).units("trend").build());
    public static final DecisionRule WX_STALE_DATA_REVIEW = rule(
            "WX-STALE-DATA-REVIEW",
            "weather-product-quality",
            "advisory",
            "Create monitor/review guidance when weather products are stale, low confidence, expired, or future issued.",
            "Weather product {productId} needs review because {warning}.",
            DecisionThreshold.builder().name("stale-age").description("Default stale threshold for aviation weather product review").comparator(">").value(30.0).units("minutes").build());
    public static final DecisionRule CONSTRAINT_COMPOUND_OVERLAP = rule(
            "CONSTRAINT-COMPOUND-OVERLAP-001",
            "compound-constraints",
            "warning",
            "Create a compound operational risk when independent constraints overlap in time, altitude, and geometry.",
            "Constraints {leftConstraint} and {rightConstraint} overlap in {dimensions}.",
            DecisionThreshold.builder().name("overlap-dimensions").description("Time, altitude, and geometry must all overlap").comparator("contains").value(3.0).units("dimensions").build());
    public static final DecisionRule WX_SEVERITY_WEIGHT = rule(
            "WX-SEVERITY-WEIGHT-001",
            "route-impact",
            "info",
            "Map hazard severity to a deterministic base route-impact weight.",
            "Severity {severity} contributes base weight {weight}.");
    public static final DecisionRule WX_LEAD_TIME_CONFIDENCE = rule(
            "WX-LEAD-TIME-CONFIDENCE-001",
            "route-impact",
            "info",
            "Degrade route-impact confidence as forecast lead time increases.",
            "Forecast hour {forecastHour} adjusted confidence to {confidence}.");
    public static final DecisionRule WX_ALTITUDE_OVERLAP = rule(
            "WX-ALTITUDE-OVERLAP-001",
            "route-impact",
            "info",
            "Only score hazards whose altitude band overlaps the route altitude band.",
            "Altitude overlap gate for {hazardId} returned {overlaps}.");
    public static final DecisionRule WX_SECTOR_CAPACITY = rule(
            "WX-SECTOR-CAPACITY-001",
            "capacity-impact",
            "warning",
            "Increase capacity impact when blocked segments affect high-demand constrained sectors.",
            "Sector {sectorId} demand/capacity ratio contributed {impact}.");
    public static final DecisionRule PARSE_RETAINED_RAW = rule(
            "WX-PARSE-RETAINED-RAW-001",
            "parsing",
            "advisory",
            "Retain recognized but unsupported weather text with diagnostics instead of throwing.",
            "Weather text was retained as classified-only for later human review.");
    public static final DecisionRule PIREP_REQUIRED_FIELDS = rule(
            "PIREP-REQUIRED-FIELDS-001",
            "pirep-quality",
            "warning",
            "PIREP reports require location, altitude, and a recognized phenomenon before they can become operational constraints.",
            "PIREP {pirepId} is missing required operational fields.");

    private static final List<DecisionRule> ALL = Collections.unmodifiableList(Arrays.asList(
            OP_ACTION_PRECEDENCE,
            WX_BLOCKED_PROBABILITY,
            WX_ECHO_TOPS_ROUTE_ALTITUDE,
            WX_GROWTH_DECAY_TREND,
            WX_STALE_DATA_REVIEW,
            CONSTRAINT_COMPOUND_OVERLAP,
            WX_SEVERITY_WEIGHT,
            WX_LEAD_TIME_CONFIDENCE,
            WX_ALTITUDE_OVERLAP,
            WX_SECTOR_CAPACITY,
            PARSE_RETAINED_RAW,
            PIREP_REQUIRED_FIELDS));

    private DecisionRuleCatalog() {
    }

    public static List<DecisionRule> all() {
        return ALL;
    }

    public static String version() {
        return VERSION;
    }

    public static java.util.Map<String, DecisionRule> rulesById() {
        java.util.Map<String, DecisionRule> rules = new java.util.LinkedHashMap<>();
        for (DecisionRule rule : ALL) {
            rules.put(rule.getId(), rule);
        }
        return java.util.Collections.unmodifiableMap(rules);
    }

    public static DecisionRule byId(String id) {
        for (DecisionRule rule : ALL) {
            if (rule.getId().equals(id)) {
                return rule;
            }
        }
        return null;
    }

    private static DecisionRule rule(String id, String category, String severity, String description,
                                     String defaultRationaleTemplate, DecisionThreshold... thresholds) {
        return DecisionRule.builder()
                .id(id)
                .category(category)
                .severity(severity)
                .description(description)
                .defaultRationaleTemplate(defaultRationaleTemplate)
                .defaultThresholds(thresholds == null ? Collections.emptyList() : Arrays.asList(thresholds))
                .build();
    }
}
