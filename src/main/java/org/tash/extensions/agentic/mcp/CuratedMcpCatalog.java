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
    public static final String FLIGHTRADAR24_MCP = "flightradar24-fr24api-mcp";

    @ConfigProperty(name = "airspace.agentic.mcp.enabled", defaultValue = "true")
    boolean mcpEnabled = true;
    @ConfigProperty(name = "airspace.agentic.mcp.external.enabled", defaultValue = "false")
    boolean externalMcpEnabled = false;
    @ConfigProperty(name = "airspace.agentic.mcp.fr24.enabled", defaultValue = "false")
    boolean fr24McpEnabled = false;

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
        servers.add(McpServerDefinition.builder()
                .id(FLIGHTRADAR24_MCP)
                .name("Flightradar24 API MCP")
                .description("Official Flightradar24 API MCP server for live and historical flight positions, summaries, tracks, airline, and airport reference data. Disabled until FR24 credentials, consent, egress policy, and review approval are configured.")
                .enabled(fr24McpEnabled)
                .external(true)
                .local(false)
                .setupRequired(!fr24McpEnabled)
                .credentialsRequired(true)
                .transport("stdio-npx")
                .version("@flightradar24/fr24api-mcp")
                .diagnostics(fr24McpEnabled
                        ? Collections.singletonList("FR24 MCP is enabled by config; FR24_API_KEY and external consent remain required per call.")
                        : Collections.singletonList("FR24 MCP is setup-required by default. Configure FR24_API_KEY and explicit egress approval before use."))
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
        if (FLIGHTRADAR24_MCP.equals(normalized)) {
            return flightradar24Tools();
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

    private List<McpToolDescriptor> flightradar24Tools() {
        return Arrays.asList(
                fr24Tool("get_live_flights_positions_light", "FR24 live positions light",
                        "Get live aircraft positions with basic latitude, longitude, altitude, speed, and heading fields. At least one filter such as bounds, flights, callsigns, airports, routes, aircraft, altitude_ranges, or categories is required.",
                        args("bounds", "flights", "callsigns", "registrations", "airports", "routes", "aircraft", "altitude_ranges", "categories", "limit"),
                        Collections.emptyList(), families("FLIGHT", "TRAFFIC", "POSITION")),
                fr24Tool("get_live_flights_positions_full", "FR24 live positions full",
                        "Get live aircraft positions with additional flight, route, aircraft, origin, and destination details. At least one filter is required.",
                        args("bounds", "flights", "callsigns", "registrations", "airports", "routes", "aircraft", "altitude_ranges", "categories", "limit"),
                        Collections.emptyList(), families("FLIGHT", "TRAFFIC", "POSITION", "AIRPORT")),
                fr24Tool("get_live_flights_count", "FR24 live flight count",
                        "Count live flights matching a bounded flight-position query before requesting detailed positions.",
                        args("bounds", "flights", "callsigns", "registrations", "airports", "routes", "aircraft", "altitude_ranges", "categories"),
                        Collections.emptyList(), families("FLIGHT", "TRAFFIC")),
                fr24Tool("get_historic_flights_positions_full", "FR24 historic positions full",
                        "Get historical aircraft position snapshots with full details for a Unix timestamp and at least one additional filter.",
                        args("timestamp", "bounds", "flights", "callsigns", "registrations", "airports", "routes", "aircraft", "altitude_ranges", "limit"),
                        Collections.singletonList("timestamp"), families("FLIGHT", "TRAFFIC", "POSITION", "REPLAY")),
                fr24Tool("get_historic_flights_positions_light", "FR24 historic positions light",
                        "Get historical aircraft position snapshots with light details for a Unix timestamp and at least one additional filter.",
                        args("timestamp", "bounds", "flights", "callsigns", "registrations", "airports", "routes", "aircraft", "altitude_ranges", "limit"),
                        Collections.singletonList("timestamp"), families("FLIGHT", "TRAFFIC", "POSITION", "REPLAY")),
                fr24Tool("get_historic_flights_count", "FR24 historic flight count",
                        "Count historical aircraft position snapshots for a Unix timestamp and bounded replay filter before requesting detailed positions.",
                        args("timestamp", "bounds", "flights", "callsigns", "registrations", "airports", "routes", "aircraft", "altitude_ranges"),
                        Collections.singletonList("timestamp"), families("FLIGHT", "TRAFFIC", "REPLAY")),
                fr24Tool("get_flight_summary_full", "FR24 flight summary full",
                        "Get detailed takeoff/landing and route summaries for a bounded flight datetime range and at least one flight, airport, route, aircraft, callsign, or registration filter.",
                        args("flight_datetime_from", "flight_datetime_to", "flights", "callsigns", "registrations", "airports", "routes", "aircraft", "sort", "limit"),
                        args("flight_datetime_from", "flight_datetime_to"), families("FLIGHT", "SCHEDULE", "AIRPORT")),
                fr24Tool("get_flight_summary_light", "FR24 flight summary light",
                        "Get essential takeoff/landing and route summaries for a bounded flight datetime range and at least one flight, airport, route, aircraft, callsign, or registration filter.",
                        args("flight_datetime_from", "flight_datetime_to", "flights", "callsigns", "registrations", "airports", "routes", "aircraft", "sort", "limit"),
                        args("flight_datetime_from", "flight_datetime_to"), families("FLIGHT", "SCHEDULE", "AIRPORT")),
                fr24Tool("get_flight_summary_count", "FR24 flight summary count",
                        "Count flight summaries for a bounded flight datetime range and at least one flight, airport, route, aircraft, callsign, or registration filter.",
                        args("flight_datetime_from", "flight_datetime_to", "flights", "callsigns", "registrations", "airports", "routes", "aircraft"),
                        args("flight_datetime_from", "flight_datetime_to"), families("FLIGHT", "SCHEDULE", "AIRPORT")),
                fr24Tool("get_flight_tracks", "FR24 flight tracks",
                        "Get positional track points for a specific Flightradar24 flight id.",
                        args("flight_id"), Collections.singletonList("flight_id"), families("FLIGHT", "TRACK", "POSITION", "REPLAY")),
                fr24Tool("get_airline_info", "FR24 airline info",
                        "Get airline reference information by ICAO code.",
                        args("icao"), Collections.singletonList("icao"), families("REFERENCE", "AIRLINE")),
                fr24Tool("get_airport_info_light", "FR24 airport info light",
                        "Get airport reference information by IATA or ICAO code.",
                        args("code"), Collections.singletonList("code"), families("REFERENCE", "AIRPORT")),
                fr24Tool("get_airport_info_full", "FR24 airport info full",
                        "Get airport reference information, location, elevation, country, city/state, and timezone by IATA or ICAO code.",
                        args("code"), Collections.singletonList("code"), families("REFERENCE", "AIRPORT")),
                fr24Tool("get_historic_flight_events_full", "FR24 historic flight events full",
                        "Get detailed selected historical flight events such as gate departure, takeoff, cruising, airspace transition, descent, landed, and gate arrival.",
                        args("flight_ids", "event_types"), args("flight_ids", "event_types"), families("FLIGHT", "EVENT", "REPLAY")),
                fr24Tool("get_historic_flight_events_light", "FR24 historic flight events light",
                        "Get selected historical flight events grouped by flight id.",
                        args("flight_ids", "event_types"), args("flight_ids", "event_types"), families("FLIGHT", "EVENT", "REPLAY"))
        );
    }

    private McpToolDescriptor fr24Tool(String id,
                                       String name,
                                       String description,
                                       List<String> arguments,
                                       List<String> requiredArguments,
                                       List<String> families) {
        Map<String, Object> schema = new LinkedHashMap<>();
        for (String argument : arguments) {
            schema.put(argument, "string");
        }
        return McpToolDescriptor.builder()
                .id(id)
                .serverId(FLIGHTRADAR24_MCP)
                .name(name)
                .description(description)
                .version("@flightradar24/fr24api-mcp")
                .enabled(fr24McpEnabled)
                .external(true)
                .setupRequired(!fr24McpEnabled)
                .credentialsRequired(true)
                .sideEffectLevel(McpSideEffectLevel.READ_ONLY)
                .riskProfile(fr24RiskProfile(id))
                .requiredArguments(requiredArguments)
                .argumentSchema(schema)
                .sourceFamilies(families)
                .diagnostics(fr24McpEnabled
                        ? Collections.singletonList("FR24 tool requires per-call external consent and FR24 API terms compliance.")
                        : Collections.singletonList("FR24 tool is setup-required by default; no live Flightradar24 call is made."))
                .build();
    }

    private AgenticRiskProfile fr24RiskProfile(String toolId) {
        return AgenticRiskProfile.builder()
                .autonomyScope("External flight-awareness lookup only; results must be normalized into Airspace replay/decision artifacts before use.")
                .toolSurface("Official Flightradar24 API MCP tool " + toolId)
                .worstCaseBlastRadius("Incorrect, stale, rate-limited, or non-authoritative commercial tracking data could mislead situational awareness if treated as official NAS truth.")
                .permissionScope("Requires FR24_API_KEY plus operator consent for each external call.")
                .dataEgress("External HTTPS egress to Flightradar24 API through the MCP server; disabled by default.")
                .poisonedDataExposure("ADS-B/commercial tracking data may be incomplete, delayed, filtered, or spoofed; source receipts and freshness must be retained.")
                .auditAttribution("MCP evidence receipt plus FR24 provider mode, query bounds/filter, and output hash when configured.")
                .toolProvenance("Official MIT-licensed Flightradar24 fr24api-mcp reference; Airspace does not vendor or execute it by default.")
                .modelProvenance("No model execution is implied; the tool is an external data lookup.")
                .rollbackPath("Disable airspace.agentic.mcp.fr24.enabled or remove FR24 MCP server configuration.")
                .costClass("EXTERNAL_API_CREDENTIAL_RATE_LIMITED")
                .slaClass("COMMERCIAL_TRACKING_PROVIDER_NON_AUTHORITATIVE")
                .requiredHumanReviewMode(HumanReviewMode.REVIEW_ONLY)
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
