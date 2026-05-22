package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@ApplicationScoped
public class MissionRiskAnalystAgent {
    @Inject
    AirspaceProductService productService;
    @Inject
    OperationalDeltaService operationalDeltaService;

    public AgentRunResult evaluate(AgentRunRequest request) {
        String missionId = missionId(request);
        ProductDtos.MissionSummary mission = productService.mission(missionId).getMission();
        ProductDtos.MissionWeatherVerdictSummary verdict = productService.missionWeatherVerdict(missionId);
        ProductDtos.RouteImpactSummary impact = productService.routeImpact(missionId, request == null ? null : request.getReservationId());
        List<ProductDtos.WeatherSourceSummary> changes = productService.weatherChanges(missionId, null, 8);
        List<AgentSourceCitation> citations = new ArrayList<>(AgentSupport.citations(impact));
        citations.add(AgentSupport.citation("MISSION", missionId, mission.getMissionNumber(), "/missions/" + missionId));
        changes.forEach(source -> citations.add(AgentSupport.citation(source.getFamily(), source.getId(), source.getLabel(), "/weather")));
        String changed = priorAction(request) == null
                ? changes.size() + " source change(s) are visible since the default brief window."
                : "Prior action " + priorAction(request) + " compared with current " + verdict.getAction()
                + "; " + changes.size() + " source change(s) are visible since the default brief window.";
        AgentFinding finding = AgentFinding.builder()
                .id(AgentSupport.id("finding", "mission-risk:" + missionId))
                .category("MISSION_RISK")
                .severity(verdict.getPriority())
                .message(mission.getMissionNumber() + " is " + verdict.getAction() + ". " + changed + " " + verdict.getSummary())
                .confidence(verdict.getConfidence())
                .citations(citations)
                .build();
        AgentFinding changeFinding = AgentFinding.builder()
                .id(AgentSupport.id("finding", "what-changed:" + missionId + ":" + changes.size()))
                .category("WHAT_CHANGED")
                .severity(changes.isEmpty() ? "INFO" : verdict.getPriority())
                .message(changes.isEmpty()
                        ? "No weather/PIREP/NOTAM deltas were detected in the default brief window."
                        : "Recent deltas: " + summarizeChanges(changes))
                .confidence(verdict.getConfidence())
                .citations(citations)
                .build();
        List<AgentOperationalDelta> deltas = operationalDeltaService == null
                ? operationalDeltas(missionId, changes)
                : operationalDeltaService.compare(request == null ? null : request.getPreviousDecisionId(),
                request == null ? null : request.getDecisionId(), missionId);
        if (deltas.isEmpty()) {
            deltas.addAll(operationalDeltas(missionId, changes));
        }
        AgentRecommendation recommendation = AgentRecommendation.builder()
                .id(AgentSupport.id("recommendation", "mission-risk:" + missionId))
                .action(verdict.getRecommendedAction())
                .summary("Recommended operator posture: " + verdict.getRecommendedAction())
                .rationale(impact.getRationale())
                .confidence(Math.min(verdict.getConfidence(), impact.getConfidence() == 0.0 ? verdict.getConfidence() : impact.getConfidence()))
                .humanApprovalRequired(true)
                .citations(citations)
                .build();
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", "mission-risk:" + missionId + ":" + verdict.getAction()))
                .agentType("MISSION_RISK")
                .summary(finding.getMessage())
                .confidence(recommendation.getConfidence())
                .accepted(true)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .findings(java.util.Arrays.asList(finding, changeFinding))
                .recommendations(java.util.Collections.singletonList(recommendation))
                .citations(citations)
                .deltas(deltas)
                .build();
    }

    private String missionId(AgentRunRequest request) {
        if (request != null && request.getMissionId() != null && !request.getMissionId().trim().isEmpty()) {
            return request.getMissionId();
        }
        List<ProductDtos.MissionSummary> missions = productService.missions();
        if (missions.isEmpty()) {
            throw new IllegalArgumentException("No mission is available for mission risk analysis");
        }
        return missions.get(0).getId();
    }

    private String priorAction(AgentRunRequest request) {
        if (request == null || request.getPreviousDecisionId() == null || request.getPreviousDecisionId().trim().isEmpty()) {
            return null;
        }
        try {
            return productService.decision(request.getPreviousDecisionId()).getAction();
        } catch (Exception ignored) {
            return "UNKNOWN";
        }
    }

    private String summarizeChanges(List<ProductDtos.WeatherSourceSummary> changes) {
        List<String> values = new ArrayList<>();
        for (ProductDtos.WeatherSourceSummary source : changes) {
            values.add(source.getFamily() + ":" + source.getId() + " " + AgentSupport.value(source.getSeverity(), "INFO"));
        }
        return String.join("; ", values);
    }

    private List<AgentOperationalDelta> operationalDeltas(String missionId, List<ProductDtos.WeatherSourceSummary> changes) {
        List<AgentOperationalDelta> deltas = new ArrayList<>();
        for (ProductDtos.WeatherSourceSummary source : changes) {
            AgentSourceCitation citation = AgentSupport.citation(source.getFamily(), source.getId(), source.getLabel(), "/weather");
            deltas.add(AgentOperationalDelta.builder()
                    .id(AgentSupport.id("delta", "source:" + missionId + ":" + source.getFamily() + ":" + source.getId()))
                    .changeType("SOURCE_OBSERVED")
                    .sourceFamily(source.getFamily())
                    .sourceId(source.getId())
                    .previousValue("not in previous brief window")
                    .currentValue(AgentSupport.value(source.getSeverity(), AgentSupport.value(source.getLabel(), source.getFamily())))
                    .severity(AgentSupport.value(source.getSeverity(), "INFO"))
                    .observedAt(source.getObservedAt())
                    .citations(java.util.Collections.singletonList(citation))
                    .build());
        }
        return deltas;
    }
}
