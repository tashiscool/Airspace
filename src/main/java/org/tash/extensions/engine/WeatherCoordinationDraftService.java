package org.tash.extensions.engine;

import java.util.ArrayList;
import java.util.List;

public class WeatherCoordinationDraftService {
    public WeatherCoordinationDraft createDraft(WeatherCoordinationDraftRequest request) {
        WeatherCoordinationDraftRequest safe = request == null ? WeatherCoordinationDraftRequest.builder().build() : request;
        String mission = safe.getMissionId() == null ? "UNKNOWN" : safe.getMissionId();
        String subject = "WEATHER COORDINATION " + mission;
        String action = safe.getRecommendedAction() == null ? "MONITOR" : safe.getRecommendedAction().name();
        List<String> sourceLines = new ArrayList<>();
        for (DecisionSourceRef ref : safe.getSourceRefs()) {
            sourceLines.add(ref.getType() + ":" + ref.getId());
        }
        String body = "SVC RQ WEATHER COORDINATION\n"
                + "MISSION " + mission + "\n"
                + "HAZARD " + (safe.getHazardOrDecisionId() == null ? "UNKNOWN" : safe.getHazardOrDecisionId()) + "\n"
                + "ACTION " + action + "\n"
                + "IMPACT " + (safe.getImpactSummary() == null ? "Review weather impact and coordinate route guidance." : safe.getImpactSummary()) + "\n"
                + "SOURCES " + String.join(", ", sourceLines);
        return WeatherCoordinationDraft.builder()
                .family("USNS/WEATHER")
                .subject(subject)
                .body(body)
                .recipients(safe.getRecipients())
                .sourceRefs(safe.getSourceRefs())
                .build();
    }
}
