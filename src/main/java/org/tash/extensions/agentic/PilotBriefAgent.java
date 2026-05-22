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
public class PilotBriefAgent {
    @Inject
    AirspaceProductService productService;

    public AgentRunResult generate(AgentRunRequest request) {
        String missionId = missionId(request);
        ProductDtos.PilotBriefSummary brief = productService.pilotBrief(missionId, null);
        List<AgentSourceCitation> citations = new ArrayList<>(AgentSupport.citations(brief.getRouteImpact()));
        brief.getChanges().forEach(source -> citations.add(AgentSupport.citation(source.getFamily(), source.getId(), source.getLabel(), "/weather")));
        AgentRecommendation recommendation = AgentRecommendation.builder()
                .id(AgentSupport.id("recommendation", "pilot-brief:" + missionId))
                .action("GENERATE_BRIEF")
                .summary("Generate pilot/controller handoff brief for " + brief.getMissionNumber())
                .rationale(brief.getDecisionTraceSummary())
                .confidence(brief.getVerdict() == null ? 0.8 : brief.getVerdict().getConfidence())
                .humanApprovalRequired(true)
                .citations(citations)
                .build();
        AgentFinding finding = AgentFinding.builder()
                .id(AgentSupport.id("finding", "brief:" + missionId))
                .category("PILOT_BRIEF")
                .severity(brief.getVerdict() == null ? "INFO" : brief.getVerdict().getPriority())
                .message("Brief ready: " + (brief.getVerdict() == null ? "No verdict" : brief.getVerdict().getAction())
                        + "; " + brief.getChanges().size() + " source change(s); " + brief.getDecisionTraceSummary())
                .confidence(recommendation.getConfidence())
                .citations(citations)
                .build();
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", "brief:" + missionId))
                .agentType("PILOT_BRIEF")
                .summary(finding.getMessage())
                .confidence(recommendation.getConfidence())
                .accepted(true)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .findings(java.util.Collections.singletonList(finding))
                .recommendations(java.util.Collections.singletonList(recommendation))
                .citations(citations)
                .build();
    }

    private String missionId(AgentRunRequest request) {
        if (request != null && request.getMissionId() != null && !request.getMissionId().trim().isEmpty()) {
            return request.getMissionId();
        }
        return productService.missions().get(0).getId();
    }
}
