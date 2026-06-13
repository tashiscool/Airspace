package org.tash.extensions.agentic.mcp;

import jakarta.enterprise.context.ApplicationScoped;
import org.eclipse.microprofile.config.inject.ConfigProperty;
import org.tash.extensions.agentic.AgenticRiskProfile;
import org.tash.extensions.agentic.HumanReviewMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

@ApplicationScoped
public class CuratedMcpCatalog implements McpToolRegistry {
    public static final String AIRSPACE_FIRST_PARTY = "airspace-first-party";
    public static final String EXTERNAL_MCP = "external-mcp";

    @ConfigProperty(name = "airspace.agentic.mcp.enabled", defaultValue = "true")
    boolean mcpEnabled = true;
    @ConfigProperty(name = "airspace.agentic.mcp.external.enabled", defaultValue = "false")
    boolean externalMcpEnabled = false;

    @Override
    public List<McpServerDefinition> servers() {
        List<McpServerDefinition> servers = new ArrayList<>();
        servers.add(McpServerDefinition.builder()
                .id(AIRSPACE_FIRST_PARTY)
                .name("Airspace first-party tools")
                .description("Local Airspace evidence and draft tools executed in-process against engine/product services.")
                .enabled(mcpEnabled)
                .external(false)
                .local(true)
                .setupRequired(false)
                .credentialsRequired(false)
                .transport("direct-java")
                .version("airspace-mcp-v1")
                .diagnostics(mcpEnabled ? Collections.emptyList() : Collections.singletonList("MCP is disabled by airspace.agentic.mcp.enabled=false"))
                .build());
        servers.add(McpServerDefinition.builder()
                .id(EXTERNAL_MCP)
                .name("External MCP servers")
                .description("Placeholder for configured stdio/SSE/HTTP MCP servers. Disabled until credentials, command allowlists, and consent are configured.")
                .enabled(externalMcpEnabled)
                .external(true)
                .local(false)
                .setupRequired(!externalMcpEnabled)
                .credentialsRequired(true)
                .transport("setup-required")
                .version("mcp-compatible")
                .diagnostics(externalMcpEnabled
                        ? Collections.singletonList("External MCP enabled by config; server-specific credentials and consent are still required per call.")
                        : Collections.singletonList("External MCP calls are setup-required by default."))
                .build());
        return servers;
    }

    @Override
    public Optional<McpServerDefinition> server(String serverId) {
        String normalized = normalize(serverId);
        return servers().stream()
                .filter(server -> normalize(server.getId()).equals(normalized))
                .findFirst();
    }

    @Override
    public List<McpToolDescriptor> tools(String serverId) {
        String normalized = normalize(serverId);
        if (AIRSPACE_FIRST_PARTY.equals(normalized)) {
            return firstPartyTools();
        }
        if (EXTERNAL_MCP.equals(normalized)) {
            return Collections.singletonList(McpToolDescriptor.builder()
                    .id("external-mcp.configured_tool")
                    .serverId(EXTERNAL_MCP)
                    .name("Configured external MCP tool")
                    .description("External MCP tool placeholder. Calls return SETUP_REQUIRED until server configuration, credentials, and consent are present.")
                    .version("mcp-compatible")
                    .enabled(externalMcpEnabled)
                    .external(true)
                    .setupRequired(!externalMcpEnabled)
                    .credentialsRequired(true)
                    .sideEffectLevel(McpSideEffectLevel.REQUIRES_APPROVAL)
                    .riskProfile(AgenticRiskProfile.builder()
                            .autonomyScope("External MCP tools are catalog placeholders only until configured.")
                            .toolSurface("Configured external MCP command/HTTP/SSE surface")
                            .worstCaseBlastRadius("Unknown external system access; blocked by default.")
                            .permissionScope("Requires explicit credentials and operator consent.")
                            .dataEgress("Potential external egress; setup-required.")
                            .poisonedDataExposure("Depends on external server and source data.")
                            .auditAttribution("MCP evidence receipt plus external server metadata when configured.")
                            .toolProvenance("External MCP provider, not bundled with Airspace.")
                            .modelProvenance("No model execution is implied by the placeholder.")
                            .rollbackPath("No external call is made in default mode; disable catalog entry if configured tool is suspect.")
                            .costClass("UNKNOWN_EXTERNAL")
                            .slaClass("UNKNOWN_EXTERNAL")
                            .requiredHumanReviewMode(HumanReviewMode.PUSH_APPROVAL)
                            .build())
                    .diagnostics(Collections.singletonList("External MCP execution is intentionally disabled by default."))
                    .build());
        }
        return Collections.emptyList();
    }

    @Override
    public Optional<McpToolDescriptor> tool(String serverId, String toolId) {
        String normalized = normalize(toolId);
        return tools(serverId).stream()
                .filter(tool -> normalize(tool.getId()).equals(normalized))
                .findFirst();
    }

    private List<McpToolDescriptor> firstPartyTools() {
        return Arrays.asList(
                tool("airspace.weather.affected_missions", "Affected missions", "List missions affected by weather, PIREP, NOTAM, or route-impact sources.", McpSideEffectLevel.READ_ONLY,
                        args("sourceId", "limit"), families("WEATHER", "PIREP", "NOTAM", "MISSION")),
                tool("airspace.mission.weather_verdict", "Mission weather verdict", "Return the current CLEAR/CAUTION/AVOID/BLOCKED style verdict for a mission.", McpSideEffectLevel.READ_ONLY,
                        args("missionId"), families("MISSION", "WEATHER", "NOTAM", "PIREP")),
                tool("airspace.mission.route_impact", "Mission route impact", "Evaluate route-impact summary for a mission and optional reservation.", McpSideEffectLevel.READ_ONLY,
                        args("missionId", "reservationId"), families("MISSION", "RESERVATION", "WEATHER", "NOTAM", "PIREP")),
                tool("airspace.pireps.relevant", "Relevant PIREPs", "Find PIREPs relevant to a mission route corridor, altitude band, and recency window.", McpSideEffectLevel.READ_ONLY,
                        args("missionId", "reservationId", "altitudeToleranceFeet", "recencyMinutes", "corridorNauticalMiles"), families("MISSION", "PIREP")),
                tool("airspace.decision.summary", "Decision summary", "Fetch a persisted operational decision summary.", McpSideEffectLevel.READ_ONLY,
                        args("decisionId"), families("DECISION")),
                tool("airspace.decision.replay_audit", "Decision replay audit", "Replay and verify a persisted decision bundle when available.", McpSideEffectLevel.READ_ONLY,
                        args("decisionId"), families("DECISION", "AUDIT")),
                tool("airspace.feed.artifact_transactions", "Feed artifact transactions", "List parsed transactions for a retained feed artifact.", McpSideEffectLevel.READ_ONLY,
                        args("feedArtifactId"), families("FEED", "USNS", "NOTAM", "WEATHER")),
                tool("airspace.reference.lookup", "Reference lookup", "Lookup reference points by type or identifier for route/weather review.", McpSideEffectLevel.READ_ONLY,
                        args("type", "identifier"), families("REFERENCE")),
                tool("airspace.coordination.draft", "Coordination draft", "Draft weather/ATC coordination text for a mission. No message is sent.", McpSideEffectLevel.DRAFT_ONLY,
                        args("missionId", "reservationId", "hazardOrDecisionId", "actor"), families("MISSION", "WEATHER", "USNS"))
        );
    }

    private McpToolDescriptor tool(String id, String name, String description, McpSideEffectLevel level,
                                   List<String> arguments, List<String> families) {
        Map<String, Object> schema = new LinkedHashMap<>();
        for (String argument : arguments) {
            schema.put(argument, "string");
        }
        return McpToolDescriptor.builder()
                .id(id)
                .serverId(AIRSPACE_FIRST_PARTY)
                .name(name)
                .description(description)
                .version("airspace-mcp-v1")
                .enabled(mcpEnabled)
                .external(false)
                .setupRequired(false)
                .credentialsRequired(false)
                .sideEffectLevel(level)
                .riskProfile(riskProfile(id, level))
                .requiredArguments(requiredArguments(id))
                .argumentSchema(schema)
                .sourceFamilies(families)
                .diagnostics(mcpEnabled ? Collections.emptyList() : Collections.singletonList("MCP is disabled by configuration."))
                .build();
    }

    private AgenticRiskProfile riskProfile(String id, McpSideEffectLevel level) {
        boolean draft = level == McpSideEffectLevel.DRAFT_ONLY;
        return AgenticRiskProfile.builder()
                .autonomyScope(draft
                        ? "Drafts coordination text from cited Airspace artifacts; cannot send or mutate workflow state."
                        : "Reads local Airspace artifacts and returns cited evidence.")
                .toolSurface(id)
                .worstCaseBlastRadius(draft
                        ? "A misleading draft could confuse an operator if approved without review; no autonomous send occurs."
                        : "Incorrect local evidence summary; no external side effects.")
                .permissionScope("Local Airspace in-process service access.")
                .dataEgress("None by default; result remains inside Airspace API response and evidence receipt.")
                .poisonedDataExposure("Could reflect stale, malformed, or adversarial local feed artifacts; output must cite source refs.")
                .auditAttribution("MCP evidence receipt with input/output hashes, policy decision, duration, and source refs.")
                .toolProvenance("Airspace first-party Java tool catalog version airspace-mcp-v1.")
                .modelProvenance("No model call; deterministic first-party service invocation.")
                .rollbackPath("Disable airspace.agentic.mcp.enabled or remove tool from curated catalog.")
                .costClass("LOCAL_CPU")
                .slaClass("LOCAL_IN_PROCESS")
                .requiredHumanReviewMode(draft ? HumanReviewMode.PUSH_APPROVAL : HumanReviewMode.REVIEW_ONLY)
                .build();
    }

    private List<String> requiredArguments(String toolId) {
        if (toolId.contains("affected_missions") || toolId.contains("reference.lookup")) {
            return Collections.emptyList();
        }
        if (toolId.contains("route_impact") || toolId.contains("pireps.relevant")
                || toolId.contains("weather_verdict") || toolId.contains("coordination.draft")) {
            return Collections.singletonList("missionId");
        }
        if (toolId.contains("decision.")) {
            return Collections.singletonList("decisionId");
        }
        if (toolId.contains("feed.")) {
            return Collections.singletonList("feedArtifactId");
        }
        return Collections.emptyList();
    }

    private List<String> args(String... values) {
        return Arrays.asList(values);
    }

    private List<String> families(String... values) {
        return Arrays.asList(values);
    }

    private String normalize(String value) {
        return value == null ? "" : value.trim().toLowerCase(Locale.US);
    }
}
