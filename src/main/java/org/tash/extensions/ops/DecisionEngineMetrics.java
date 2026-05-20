package org.tash.extensions.ops;

import org.tash.extensions.engine.OperationalDecisionResult;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public final class DecisionEngineMetrics {
    public static final String DECISION_EVALUATED = "decision.evaluated";
    public static final String CARF_CONFLICTS = "decision.carf.conflicts";
    public static final String ROUTE_BLOCKAGES = "decision.route.blockages";
    public static final String ACTION_SELECTED = "decision.action.selected";
    public static final String REPLAY_FAILURE = "decision.replay.failure";
    public static final String STAGE_DURATION = "decision.stage.duration";

    private DecisionEngineMetrics() {
    }

    public static void recordDecision(OperationalMetricSink sink, OperationalDecisionResult result) {
        if (sink == null || result == null) {
            return;
        }
        sink.increment(DECISION_EVALUATED, Collections.emptyMap());
        sink.record(metric(CARF_CONFLICTS, result.getConflicts().size(), "count", Collections.emptyMap()));
        sink.record(metric(ROUTE_BLOCKAGES, result.getRouteBlockages().stream().filter(blockage -> blockage.isBlocked()).count(),
                "count", Collections.emptyMap()));
        Map<String, String> tags = new LinkedHashMap<>();
        tags.put("action", result.getAction() == null ? "UNKNOWN" : result.getAction().name());
        sink.increment(ACTION_SELECTED, tags);
    }

    public static void recordDuration(OperationalMetricSink sink, String stage, long millis) {
        if (sink == null) {
            return;
        }
        Map<String, String> tags = new LinkedHashMap<>();
        tags.put("stage", stage);
        sink.duration(STAGE_DURATION, millis, tags);
    }

    private static OperationalMetric metric(String name, double value, String unit, Map<String, String> tags) {
        return OperationalMetric.builder()
                .name(name)
                .value(value)
                .unit(unit)
                .tags(tags)
                .timestamp(java.time.ZonedDateTime.now(java.time.ZoneOffset.UTC))
                .build();
    }
}
