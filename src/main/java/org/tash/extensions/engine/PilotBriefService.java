package org.tash.extensions.engine;

import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.decision.WeatherDecisionAction;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

public class PilotBriefService {
    public String generate(String missionId, OperationalDecisionResult result, ZonedDateTime generatedAt) {
        OperationalDecisionResult safe = result == null ? OperationalDecisionResult.builder()
                .action(WeatherDecisionAction.MONITOR)
                .rationale("No decision result was supplied")
                .confidence(0.0)
                .build() : result;
        List<String> lines = new ArrayList<>();
        lines.add("AIRSPACE PILOT BRIEF");
        lines.add("MISSION: " + (missionId == null ? "UNKNOWN" : missionId));
        lines.add("GENERATED: " + (generatedAt == null ? ZonedDateTime.now() : generatedAt));
        lines.add("VERDICT: " + safe.getAction() + " / " + safe.getRecommendedAction());
        lines.add("CONFIDENCE: " + Math.round(safe.getConfidence() * 100.0) + "%");
        lines.add("RATIONALE: " + safe.getRationale());
        if (!safe.getRouteImpacts().isEmpty()) {
            lines.add("ROUTE IMPACTS:");
            for (RouteBlockagePrediction impact : safe.getRouteImpacts()) {
                lines.add("- T+" + impact.getForecastHour() + "h " + impact.getRecommendedAction()
                        + " p=" + String.format("%.2f", impact.getBlockedProbability())
                        + " source=" + String.join(",", impact.getSourceRefs()));
            }
        }
        if (!safe.getSourceRefs().isEmpty()) {
            List<String> refs = new ArrayList<>();
            for (DecisionSourceRef ref : safe.getSourceRefs()) {
                refs.add(ref.getType() + ":" + ref.getId());
            }
            lines.add("SOURCES: " + String.join(", ", refs));
        }
        if (safe.getTrace() != null) {
            lines.add("TRACE STEPS: " + safe.getTrace().getSteps().size());
        }
        return String.join("\n", lines);
    }
}
