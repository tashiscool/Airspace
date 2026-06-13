package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.agentic.mcp.AgentToolPlan;
import org.tash.extensions.agentic.mcp.AgentToolPlanner;
import org.tash.extensions.agentic.mcp.McpToolDescriptor;
import org.tash.extensions.engine.CanonicalJson;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@ApplicationScoped
public class AgentReasoningContextBuilder {
    @Inject
    AgentToolPlanner toolPlanner;

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
        facts.add("Tool calls: " + safeDraft.getToolCalls().size());
        AgentPolicy safePolicy = policy == null ? AgentPolicy.builder().build() : policy;
        AgentToolPlan toolPlan = planner().plan(safeRequest, safePolicy);
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
                        "Do not present assistant analysis as official clearance",
                        "Only use curated MCP tools listed in availableTools; blockedTools are unavailable evidence, not permission to improvise"))
                .citations(safeDraft.getCitations())
                .availableTools(toolNames(toolPlan.getAvailableTools()))
                .blockedTools(toolNames(toolPlan.getBlockedTools()))
                .toolPolicySummary("MCP tools provide cited evidence and drafts only; official sends and workflow mutations remain human-approved.")
                .toolReceiptIds(receiptIds(safeDraft))
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

    private AgentToolPlanner planner() {
        if (toolPlanner != null) {
            return toolPlanner;
        }
        return new AgentToolPlanner();
    }

    private List<String> toolNames(List<McpToolDescriptor> tools) {
        List<String> names = new ArrayList<>();
        for (McpToolDescriptor tool : tools == null ? java.util.Collections.<McpToolDescriptor>emptyList() : tools) {
            String suffix = tool.getSideEffectLevel() == null ? "" : " [" + tool.getSideEffectLevel().name() + "]";
            names.add(tool.getId() + suffix);
        }
        return names;
    }

    private List<String> receiptIds(AgentRunResult result) {
        List<String> ids = new ArrayList<>();
        for (AgentToolCall call : result.getToolCalls()) {
            if (call.getEvidenceReceiptId() != null && !call.getEvidenceReceiptId().trim().isEmpty()) {
                ids.add(call.getEvidenceReceiptId());
            }
        }
        return ids;
    }
}
