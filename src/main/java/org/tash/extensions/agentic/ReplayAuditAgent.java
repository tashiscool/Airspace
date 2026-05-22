package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

@ApplicationScoped
public class ReplayAuditAgent {
    @Inject
    AirspaceProductService productService;
    @Inject
    OperationalDeltaService operationalDeltaService;

    public AgentRunResult explain(AgentRunRequest request) {
        String decisionId = request == null ? null : request.getDecisionId();
        ProductDtos.DecisionSummary decision = decisionId == null || decisionId.trim().isEmpty()
                ? latestDecision()
                : productService.decision(decisionId);
        List<AgentSourceCitation> citations = decision.getRouteImpact() == null
                ? java.util.Collections.singletonList(AgentSupport.citation("DECISION", decision.getId(), decision.getAction(), "/decisions/" + decision.getId()))
                : AgentSupport.citations(decision.getRouteImpact());
        AgentFinding finding = AgentFinding.builder()
                .id(AgentSupport.id("finding", "replay:" + decision.getId()))
                .category("REPLAY_AUDIT")
                .severity("INFO")
                .message(questionPrefix(request) + "Decision " + decision.getId() + " recommended " + decision.getRecommendedAction()
                        + " from action " + decision.getAction() + " at "
                        + Math.round(decision.getConfidence() * 100.0) + "% confidence. "
                        + AgentSupport.value(decision.getRationale(), "Replay bundle and audit envelope retained for verification."))
                .confidence(Math.max(0.5, decision.getConfidence()))
                .citations(citations)
                .build();
        AgentTraceAnswer answer = traceAnswer(request, decision, citations);
        List<AgentOperationalDelta> deltas = operationalDeltaService == null
                ? java.util.Collections.emptyList()
                : operationalDeltaService.compare(request == null ? null : request.getPreviousDecisionId(), decision.getId(), request == null ? null : request.getMissionId());
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", "replay:" + decision.getId()))
                .agentType("REPLAY_AUDIT")
                .summary(finding.getMessage())
                .confidence(finding.getConfidence())
                .accepted(true)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .findings(java.util.Collections.singletonList(finding))
                .citations(citations)
                .traceAnswer(answer)
                .deltas(deltas)
                .build();
    }

    private AgentTraceAnswer traceAnswer(AgentRunRequest request,
                                         ProductDtos.DecisionSummary decision,
                                         List<AgentSourceCitation> citations) {
        String question = request == null || request.getQuestion() == null || request.getQuestion().trim().isEmpty()
                ? "Why was this decision recommended?"
                : request.getQuestion().trim();
        List<String> sourceRefs = new ArrayList<>();
        Set<String> ruleIds = new LinkedHashSet<>();
        if (decision.getRouteImpact() != null) {
            sourceRefs.addAll(decision.getRouteImpact().getSourceRefs());
            for (ProductDtos.RerouteTraceSummary step : decision.getRouteImpact().getWhyRerouteTrace()) {
                if (step.getRuleId() != null && !step.getRuleId().trim().isEmpty()) {
                    ruleIds.add(step.getRuleId().trim());
                }
                if (step.getSourceRef() != null && !step.getSourceRef().trim().isEmpty()) {
                    sourceRefs.add(step.getSourceRef().trim());
                }
            }
        }
        if (sourceRefs.isEmpty()) {
            citations.stream()
                    .filter(citation -> citation.getSourceId() != null)
                    .forEach(citation -> sourceRefs.add(citation.getSourceFamily() + ":" + citation.getSourceId()));
        }
        String answer = "Decision " + decision.getId() + " is "
                + AgentSupport.value(decision.getRecommendedAction(), decision.getAction())
                + " at " + Math.round(decision.getConfidence() * 100.0) + "% confidence because "
                + AgentSupport.value(decision.getRationale(), "the retained decision trace and route-impact artifacts support that action")
                + ". Source refs: " + (sourceRefs.isEmpty() ? "none supplied" : String.join(", ", sourceRefs))
                + ". Rule IDs: " + (ruleIds.isEmpty() ? "none supplied" : String.join(", ", ruleIds)) + ".";
        List<String> unsupported = sourceRefs.isEmpty()
                ? java.util.Collections.singletonList("Trace answer has no explicit source refs beyond the decision citation")
                : java.util.Collections.emptyList();
        return AgentTraceAnswer.builder()
                .question(question)
                .answer(answer)
                .confidence(Math.max(0.5, decision.getConfidence()))
                .sourceRefs(sourceRefs)
                .ruleIds(new ArrayList<>(ruleIds))
                .unsupportedClaims(unsupported)
                .citations(citations)
                .build();
    }

    private String questionPrefix(AgentRunRequest request) {
        if (request == null || request.getQuestion() == null || request.getQuestion().trim().isEmpty()) {
            return "";
        }
        return "Trace question: \"" + request.getQuestion().trim() + "\". Evidence-grounded answer: ";
    }

    private ProductDtos.DecisionSummary latestDecision() {
        ProductDtos.DecisionEvaluateRequest request = new ProductDtos.DecisionEvaluateRequest();
        if (!productService.missions().isEmpty()) {
            request.setMissionId(productService.missions().get(0).getId());
        }
        return productService.evaluateDecision(request);
    }
}
