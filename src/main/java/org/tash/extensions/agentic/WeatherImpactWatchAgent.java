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
public class WeatherImpactWatchAgent {
    @Inject
    AirspaceProductService productService;

    public AgentRunResult evaluate(AgentRunRequest request) {
        List<ProductDtos.AffectedMissionSummary> affected = productService.affectedMissions(
                request == null ? null : request.getHazardOrDecisionId(), 25);
        List<AgentFinding> findings = new ArrayList<>();
        List<AgentTask> tasks = new ArrayList<>();
        for (ProductDtos.AffectedMissionSummary mission : affected) {
            List<AgentSourceCitation> citations = AgentSupport.citations(mission.getSourceRefs());
            citations.add(AgentSupport.citation("MISSION", mission.getMissionId(), mission.getMissionNumber(), "/missions/" + mission.getMissionId()));
            String severity = mission.getBlockingConstraintCount() > 0 ? "HIGH" : "MEDIUM";
            findings.add(AgentFinding.builder()
                    .id(AgentSupport.id("finding", "weather-impact:" + mission.getMissionId()))
                    .category("WEATHER_IMPACT")
                    .severity(severity)
                    .message(mission.getMissionNumber() + " is " + mission.getAction()
                            + " with " + mission.getImpactedSegmentCount() + " impacted segment(s): " + mission.getRationale())
                    .confidence(mission.getConfidence())
                    .citations(citations)
                    .build());
            tasks.add(AgentTask.builder()
                    .id(AgentSupport.id("task", "review-weather:" + mission.getMissionId()))
                    .title("Review weather impact for " + mission.getMissionNumber())
                    .status("OPEN")
                    .priority(mission.getPriority())
                    .assignedRole("PLANNER")
                    .route("/missions/" + mission.getMissionId())
                    .rationale("Affected mission requires operator review before coordination or pilot handoff.")
                    .citations(citations)
                    .build());
        }
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", "weather-impact:" + ZonedDateTime.now(ZoneOffset.UTC).toEpochSecond()))
                .agentType("WEATHER_IMPACT")
                .summary(affected.isEmpty()
                        ? "No currently affected missions were detected from retained weather, PIREP, NOTAM, and route-impact artifacts."
                        : affected.size() + " affected mission(s) need weather-impact review.")
                .confidence(affected.isEmpty() ? 0.9 : affected.stream().mapToDouble(ProductDtos.AffectedMissionSummary::getConfidence).average().orElse(0.75))
                .accepted(true)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .findings(findings)
                .tasks(tasks)
                .citations(findings.isEmpty() ? java.util.Collections.singletonList(AgentSupport.citation("ENGINE", "affected-missions", "Affected mission query", "/weather")) : findings.get(0).getCitations())
                .build();
    }
}
