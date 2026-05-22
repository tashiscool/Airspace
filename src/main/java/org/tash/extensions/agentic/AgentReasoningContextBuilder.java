package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import org.tash.extensions.engine.CanonicalJson;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@ApplicationScoped
public class AgentReasoningContextBuilder {
    public AgentReasoningEnvelope build(AgentRunRequest request, AgentRunResult draft, AgentPolicy policy) {
        AgentRunRequest safeRequest = request == null ? new AgentRunRequest() : request;
        AgentRunResult safeDraft = draft == null ? AgentRunResult.builder().id("missing-draft").agentType("UNKNOWN").build() : draft;
        String draftJson = CanonicalJson.write(safeDraft.toBuilder()
                .auditEnvelope(null)
                .reasoningEnvelope(null)
                .build());
        List<String> facts = new ArrayList<>();
        facts.add("Agent type: " + safeDraft.getAgentType());
        facts.add("Mission id: " + value(safeRequest.getMissionId(), "not supplied"));
        facts.add("Reservation id: " + value(safeRequest.getReservationId(), "not supplied"));
        facts.add("Decision id: " + value(safeRequest.getDecisionId(), "not supplied"));
        facts.add("Findings: " + safeDraft.getFindings().size());
        facts.add("Recommendations: " + safeDraft.getRecommendations().size());
        facts.add("Tasks: " + safeDraft.getTasks().size());
        facts.add("Deltas: " + safeDraft.getDeltas().size());
        AgentPolicy safePolicy = policy == null ? AgentPolicy.builder().build() : policy;
        return AgentReasoningEnvelope.builder()
                .id(AgentSupport.id("reasoning", safeDraft.getId() + ":" + CanonicalJson.sha256(draftJson)))
                .promptVersion("agentic-nextgen-v1")
                .modelId("deterministic-draft-only")
                .reasoningMode("DETERMINISTIC_DRAFT_ONLY")
                .inputSummary(inputSummary(safeRequest, safeDraft))
                .draftHash(CanonicalJson.sha256(draftJson))
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .allowedFacts(facts)
                .requiredOutputRules(Arrays.asList(
                        "Use only the supplied typed artifacts and citations",
                        "Every operational claim must cite a source",
                        "State when evidence is missing",
                        "Keep deterministic engine action authoritative"))
                .prohibitedActions(Arrays.asList(
                        safePolicy.isAllowExternalSend() ? "No autonomous external send unless policy explicitly permits it" : "External sends prohibited",
                        safePolicy.isAllowOfficialStateMutation() ? "No official workflow mutation unless policy explicitly permits it" : "Official workflow mutation prohibited",
                        "Do not invent weather, route, NOTAM, PIREP, CARF, recipient, or cost facts",
                        "Do not present assistant analysis as official clearance"))
                .citations(safeDraft.getCitations())
                .build();
    }

    private String inputSummary(AgentRunRequest request, AgentRunResult draft) {
        return "Agent " + value(draft.getAgentType(), "UNKNOWN")
                + " over mission " + value(request.getMissionId(), "not supplied")
                + ", reservation " + value(request.getReservationId(), "not supplied")
                + ", decision " + value(request.getDecisionId(), "not supplied")
                + " with " + draft.getFindings().size() + " finding(s), "
                + draft.getRecommendations().size() + " recommendation(s), "
                + draft.getTasks().size() + " task(s), and "
                + draft.getCitations().size() + " top-level citation(s).";
    }

    private String value(String value, String fallback) {
        return value == null || value.trim().isEmpty() ? fallback : value.trim();
    }
}
