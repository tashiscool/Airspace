package org.tash.extensions.agentic.mcp;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.agentic.AgentPolicy;
import org.tash.extensions.agentic.AgentRunRequest;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

@ApplicationScoped
public class AgentToolPlanner {
    @Inject
    CuratedMcpCatalog catalog;
    @Inject
    McpPolicyGate policyGate;

    public AgentToolPlan plan(AgentRunRequest request, AgentPolicy policy) {
        AgentRunRequest safe = request == null ? new AgentRunRequest() : request;
        String type = safe.getAgentType() == null ? "ALL" : safe.getAgentType().trim().toUpperCase(Locale.US);
        Set<String> ids = new LinkedHashSet<>(toolIdsFor(type));
        CuratedMcpCatalog safeCatalog = catalog == null ? new CuratedMcpCatalog() : catalog;
        McpPolicyGate safeGate = policyGate == null ? new McpPolicyGate() : policyGate;
        List<McpToolDescriptor> available = new ArrayList<>();
        List<McpToolDescriptor> blocked = new ArrayList<>();
        List<String> diagnostics = new ArrayList<>();
        McpServerDefinition server = safeCatalog.server(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY).orElse(null);
        for (String id : ids) {
            McpToolDescriptor descriptor = safeCatalog.tool(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY, id).orElse(null);
            if (descriptor == null) {
                diagnostics.add("Planned MCP tool is not in curated catalog: " + id);
                continue;
            }
            McpInvocationStatus status = safeGate.validate(server, descriptor, invocationForPlan(descriptor), policy);
            if (status == McpInvocationStatus.ACCEPTED && contextAvailable(descriptor, safe)) {
                available.add(descriptor);
            } else {
                blocked.add(descriptor.toBuilder()
                        .diagnostics(Collections.singletonList(blockReason(status, descriptor, safe)))
                        .build());
            }
        }
        for (McpToolDescriptor external : safeCatalog.tools(CuratedMcpCatalog.EXTERNAL_MCP)) {
            blocked.add(external);
        }
        return AgentToolPlan.builder()
                .availableTools(available)
                .blockedTools(blocked)
                .diagnostics(diagnostics)
                .build();
    }

    private List<String> toolIdsFor(String type) {
        switch (type) {
            case "WEATHER_IMPACT":
                return Arrays.asList("airspace.weather.affected_missions", "airspace.mission.route_impact", "airspace.pireps.relevant");
            case "MISSION_RISK":
                return Arrays.asList("airspace.mission.weather_verdict", "airspace.mission.route_impact", "airspace.decision.summary");
            case "REROUTE_ANALYSIS":
                return Arrays.asList("airspace.mission.route_impact", "airspace.reference.lookup");
            case "COORDINATION_DRAFT":
                return Arrays.asList("airspace.coordination.draft", "airspace.mission.route_impact");
            case "PILOT_BRIEF":
                return Arrays.asList("airspace.mission.weather_verdict", "airspace.mission.route_impact", "airspace.pireps.relevant", "airspace.coordination.draft");
            case "DATA_INTEGRITY":
                return Arrays.asList("airspace.feed.artifact_transactions", "airspace.reference.lookup", "airspace.mission.weather_verdict");
            case "REPLAY_AUDIT":
                return Arrays.asList("airspace.decision.summary", "airspace.decision.replay_audit");
            case "ALL":
            default:
                return Arrays.asList(
                        "airspace.weather.affected_missions",
                        "airspace.mission.weather_verdict",
                        "airspace.mission.route_impact",
                        "airspace.pireps.relevant",
                        "airspace.decision.summary",
                        "airspace.decision.replay_audit",
                        "airspace.feed.artifact_transactions",
                        "airspace.reference.lookup",
                        "airspace.coordination.draft");
        }
    }

    private McpToolInvocationRequest invocationForPlan(McpToolDescriptor descriptor) {
        return McpToolInvocationRequest.builder()
                .serverId(descriptor.getServerId())
                .toolId(descriptor.getId())
                .externalConsent(false)
                .build();
    }

    private boolean contextAvailable(McpToolDescriptor descriptor, AgentRunRequest request) {
        for (String required : descriptor.getRequiredArguments()) {
            if ("missionId".equals(required) && blank(request.getMissionId())) return false;
            if ("decisionId".equals(required) && blank(request.getDecisionId())) return false;
            if ("feedArtifactId".equals(required) && blank(request.getFeedArtifactId())) return false;
        }
        return true;
    }

    private String blockReason(McpInvocationStatus status, McpToolDescriptor descriptor, AgentRunRequest request) {
        if (status == McpInvocationStatus.SETUP_REQUIRED) {
            return "Tool setup is required before invocation.";
        }
        if (!contextAvailable(descriptor, request)) {
            return "Required request context is missing: " + descriptor.getRequiredArguments();
        }
        return "Tool blocked by agent policy.";
    }

    private boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }
}
