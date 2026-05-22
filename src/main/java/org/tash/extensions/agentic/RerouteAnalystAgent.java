package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.Comparator;
import java.util.List;

@ApplicationScoped
public class RerouteAnalystAgent {
    @Inject
    AirspaceProductService productService;

    public AgentRunResult evaluate(AgentRunRequest request) {
        String missionId = missionId(request);
        ProductDtos.RouteImpactSummary impact = productService.routeImpact(missionId, request == null ? null : request.getReservationId());
        ProductDtos.RouteCandidateComparisonSummary best = impact.getCandidateComparisons().stream()
                .min(Comparator.comparingInt((ProductDtos.RouteCandidateComparisonSummary candidate) -> candidate.getResidualConstraints().size())
                        .thenComparingDouble(candidate -> candidate.getCost() == null ? Double.POSITIVE_INFINITY : candidate.getCost().getAdditionalCostUsd()))
                .orElse(null);
        List<AgentSourceCitation> citations = AgentSupport.citations(impact);
        AgentFinding finding = AgentFinding.builder()
                .id(AgentSupport.id("finding", "reroute:" + missionId))
                .category("REROUTE_ANALYSIS")
                .severity(impact.getBlockingConstraintCount() > 0 ? "HIGH" : "MEDIUM")
                .message(best == null
                        ? "No route candidate is currently available; operator review remains required. " + impact.getRationale()
                        : best.getLabel() + " is the current best candidate: +" + Math.round(best.getCost().getAdditionalDistanceNm())
                        + " NM, +" + Math.round(best.getCost().getAdditionalMinutes()) + " min, +$"
                        + Math.round(best.getCost().getAdditionalCostUsd()) + ", avoiding "
                        + best.getAvoidedConstraints().size() + " constraint(s) with "
                        + best.getResidualConstraints().size() + " residual constraint(s).")
                .confidence(best == null ? impact.getConfidence() : best.getConfidence())
                .citations(citations)
                .build();
        AgentRecommendation recommendation = AgentRecommendation.builder()
                .id(AgentSupport.id("recommendation", "reroute:" + missionId))
                .action(best == null ? "REVIEW" : "REROUTE_REVIEW")
                .summary(best == null ? "Review route manually" : "Review " + best.getLabel() + " for operational use")
                .rationale(best == null ? impact.getRationale() : AgentSupport.value(best.getRationale(), "Candidate minimizes residual constraints and cost delta."))
                .confidence(finding.getConfidence())
                .humanApprovalRequired(true)
                .citations(citations)
                .build();
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", "reroute:" + missionId + ":" + impact.getAction()))
                .agentType("REROUTE_ANALYSIS")
                .summary(finding.getMessage())
                .confidence(finding.getConfidence())
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
