package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.List;

@ApplicationScoped
public class CoordinationDraftAgent {
    @Inject
    AirspaceProductService productService;

    public AgentRunResult createDraft(AgentRunRequest request) {
        String missionId = missionId(request);
        ProductDtos.CoordinationDraftRequest draftRequest = new ProductDtos.CoordinationDraftRequest();
        draftRequest.setMissionId(missionId);
        draftRequest.setReservationId(request == null ? null : request.getReservationId());
        draftRequest.setHazardOrDecisionId(request == null ? null : request.getHazardOrDecisionId());
        draftRequest.setActor(request == null ? null : request.getActor());
        ProductDtos.CoordinationDraftSummary draft = productService.coordinationDraft(missionId, draftRequest);
        List<AgentSourceCitation> citations = AgentSupport.citations(draft.getSourceRefs());
        AgentRecommendation recommendation = AgentRecommendation.builder()
                .id(AgentSupport.id("recommendation", "coordination:" + draft.getId()))
                .action("DRAFT_COORDINATION")
                .summary("Draft " + draft.getFamily() + " coordination message: " + draft.getSubject())
                .rationale("Draft is prepared from mission verdict, route impact, and retained source references. Human approval is required before send.")
                .confidence(0.86)
                .humanApprovalRequired(true)
                .citations(citations)
                .build();
        AgentTask task = AgentTask.builder()
                .id(AgentSupport.id("task", "approve-coordination:" + draft.getId()))
                .title("Review and approve coordination draft")
                .status("DRAFT")
                .priority("MEDIUM")
                .assignedRole("PLANNER")
                .route("/messages")
                .rationale("Agent output is a draft only; operator must approve and send.")
                .citations(citations)
                .build();
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", "coordination:" + draft.getId()))
                .agentType("COORDINATION_DRAFT")
                .summary("Coordination draft created for " + draft.getMissionId() + "; send is blocked until operator approval.")
                .confidence(0.86)
                .accepted(true)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .recommendations(java.util.Collections.singletonList(recommendation))
                .tasks(java.util.Collections.singletonList(task))
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
