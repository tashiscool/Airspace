package org.tash.extensions.agentic.mcp;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.agentic.AgentPolicy;
import org.tash.extensions.agentic.AgentSourceCitation;
import org.tash.extensions.agentic.AgentToolCall;
import org.tash.extensions.engine.CanonicalJson;
import org.tash.extensions.engine.OperationalDecisionEngine;
import org.tash.extensions.engine.ReplayVerificationResult;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.time.Duration;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

@ApplicationScoped
public class CuratedMcpInvocationService {
    @Inject
    CuratedMcpCatalog catalog;
    @Inject
    McpPolicyGate policyGate;
    @Inject
    McpRedactor redactor;
    @Inject
    McpReceiptStore receiptStore;
    @Inject
    AirspaceProductService productService;

    public McpToolInvocationResult call(McpToolInvocationRequest request) {
        McpToolInvocationRequest safe = request == null ? new McpToolInvocationRequest() : request;
        AgentPolicy policy = safe.getPolicy() == null ? AgentPolicy.builder().build() : safe.getPolicy();
        ZonedDateTime started = ZonedDateTime.now(ZoneOffset.UTC);
        CuratedMcpCatalog safeCatalog = catalog == null ? new CuratedMcpCatalog() : catalog;
        McpPolicyGate safeGate = policyGate == null ? new McpPolicyGate() : policyGate;
        McpRedactor safeRedactor = redactor == null ? new McpRedactor() : redactor;
        Optional<McpServerDefinition> server = safeCatalog.server(safe.getServerId());
        Optional<McpToolDescriptor> tool = safeCatalog.tool(safe.getServerId(), safe.getToolId());
        List<String> diagnostics = new ArrayList<>();
        if (server.isEmpty()) {
            diagnostics.add("Unknown MCP server: " + safe.getServerId());
            return denied(safe, null, started, diagnostics, safeRedactor, "UNKNOWN_SERVER");
        }
        if (tool.isEmpty()) {
            diagnostics.add("Unknown MCP tool: " + safe.getToolId());
            return denied(safe, server.get(), started, diagnostics, safeRedactor, "UNKNOWN_TOOL");
        }
        McpToolDescriptor descriptor = tool.get();
        McpInvocationStatus policyStatus = safeGate.validate(server.get(), descriptor, safe, policy);
        if (policyStatus != McpInvocationStatus.ACCEPTED) {
            diagnostics.add(policyStatus == McpInvocationStatus.SETUP_REQUIRED
                    ? "MCP tool is setup-required: " + descriptor.getId()
                    : "MCP tool denied by policy: " + descriptor.getId());
            return completed(safe, server.get(), descriptor, policyStatus, safeGate.decision(policyStatus, descriptor),
                    started, null, diagnostics, safeRedactor);
        }
        List<String> missing = missingArguments(descriptor, safe.getArguments());
        if (!missing.isEmpty()) {
            diagnostics.add("Missing required MCP argument(s): " + String.join(", ", missing));
            return completed(safe, server.get(), descriptor, McpInvocationStatus.DENIED,
                    "DENIED_MISSING_ARGUMENTS", started, null, diagnostics, safeRedactor);
        }
        try {
            Object result = executeFirstParty(descriptor.getId(), safe.getArguments());
            return completed(safe, server.get(), descriptor, McpInvocationStatus.ACCEPTED,
                    safeGate.decision(McpInvocationStatus.ACCEPTED, descriptor), started, result, diagnostics, safeRedactor);
        } catch (Exception ex) {
            diagnostics.add("MCP tool failed: " + ex.getClass().getSimpleName() + ": " + ex.getMessage());
            return completed(safe, server.get(), descriptor, McpInvocationStatus.FAILED,
                    "FAILED_RUNTIME_EXCEPTION", started, null, diagnostics, safeRedactor);
        }
    }

    public List<McpEvidenceReceipt> receipts(Integer limit) {
        return store().list(limit);
    }

    private Object executeFirstParty(String toolId, Map<String, Object> arguments) {
        AirspaceProductService service = productService;
        if (service == null) {
            throw new IllegalStateException("AirspaceProductService is not configured");
        }
        switch (toolId) {
            case "airspace.weather.affected_missions":
                return service.affectedMissions(string(arguments, "sourceId"), integer(arguments, "limit"));
            case "airspace.mission.weather_verdict":
                return service.missionWeatherVerdict(required(arguments, "missionId"));
            case "airspace.mission.route_impact":
                return service.routeImpact(required(arguments, "missionId"), string(arguments, "reservationId"));
            case "airspace.pireps.relevant":
                ProductDtos.PirepRelevanceRequest pirep = new ProductDtos.PirepRelevanceRequest();
                pirep.setReservationId(string(arguments, "reservationId"));
                pirep.setAltitudeToleranceFeet(number(arguments, "altitudeToleranceFeet", 2000.0));
                pirep.setRecencyMinutes(integer(arguments, "recencyMinutes", 60) == null ? 60 : integer(arguments, "recencyMinutes", 60));
                pirep.setCorridorNauticalMiles(number(arguments, "corridorNauticalMiles", 40.0));
                return service.relevantPireps(required(arguments, "missionId"), pirep);
            case "airspace.decision.summary":
                return service.decision(required(arguments, "decisionId"));
            case "airspace.decision.replay_audit":
                ReplayVerificationResult replay = new OperationalDecisionEngine()
                        .replay(service.decisionReplayBundle(required(arguments, "decisionId")));
                return replay;
            case "airspace.feed.artifact_transactions":
                return service.feedTransactions(required(arguments, "feedArtifactId"));
            case "airspace.reference.lookup":
                return referenceLookup(service, arguments);
            case "airspace.coordination.draft":
                ProductDtos.CoordinationDraftRequest draft = new ProductDtos.CoordinationDraftRequest();
                draft.setMissionId(required(arguments, "missionId"));
                draft.setReservationId(string(arguments, "reservationId"));
                draft.setHazardOrDecisionId(string(arguments, "hazardOrDecisionId"));
                draft.setActor(string(arguments, "actor"));
                return service.coordinationDraft(required(arguments, "missionId"), draft);
            default:
                throw new IllegalArgumentException("Unsupported first-party MCP tool: " + toolId);
        }
    }

    private Object referenceLookup(AirspaceProductService service, Map<String, Object> arguments) {
        String identifier = string(arguments, "identifier");
        String type = string(arguments, "type");
        List<ProductDtos.ReferencePointSummary> points = service.referencePoints(type);
        if (identifier == null || identifier.trim().isEmpty()) {
            return points;
        }
        String expected = identifier.trim().toUpperCase(Locale.US);
        List<ProductDtos.ReferencePointSummary> matches = new ArrayList<>();
        for (ProductDtos.ReferencePointSummary point : points) {
            if (point.getIdentifier() != null && expected.equals(point.getIdentifier().trim().toUpperCase(Locale.US))) {
                matches.add(point);
            }
        }
        return matches;
    }

    private McpToolInvocationResult denied(McpToolInvocationRequest request,
                                           McpServerDefinition server,
                                           ZonedDateTime started,
                                           List<String> diagnostics,
                                           McpRedactor safeRedactor,
                                           String policyDecision) {
        McpToolDescriptor descriptor = McpToolDescriptor.builder()
                .id(request == null ? null : request.getToolId())
                .serverId(server == null ? null : server.getId())
                .sideEffectLevel(McpSideEffectLevel.READ_ONLY)
                .build();
        return completed(request, server, descriptor, McpInvocationStatus.DENIED, policyDecision, started, null, diagnostics, safeRedactor);
    }

    private McpToolInvocationResult completed(McpToolInvocationRequest request,
                                              McpServerDefinition server,
                                              McpToolDescriptor tool,
                                              McpInvocationStatus status,
                                              String policyDecision,
                                              ZonedDateTime started,
                                              Object result,
                                              List<String> diagnostics,
                                              McpRedactor safeRedactor) {
        ZonedDateTime completed = ZonedDateTime.now(ZoneOffset.UTC);
        McpRedactionResult redactedInput = safeRedactor.redact(request == null ? Collections.emptyMap() : request.getArguments());
        McpRedactionResult redactedOutput = safeRedactor.redact(result);
        String inputHash = CanonicalJson.sha256(redactedInput.getValue());
        String outputHash = CanonicalJson.sha256(redactedOutput.getValue());
        List<AgentSourceCitation> sourceRefs = sourceRefs(result, tool);
        String receiptId = "mcp-receipt-" + java.util.UUID.nameUUIDFromBytes((
                value(server == null ? null : server.getId()) + ":"
                        + value(tool == null ? null : tool.getId()) + ":"
                        + status + ":" + inputHash + ":" + outputHash).getBytes(java.nio.charset.StandardCharsets.UTF_8));
        String redactionStatus = "REDACTED".equals(redactedInput.getStatus()) || "REDACTED".equals(redactedOutput.getStatus())
                ? "REDACTED"
                : "CLEAN";
        McpEvidenceReceipt receipt = McpEvidenceReceipt.builder()
                .id(receiptId)
                .serverId(server == null ? null : server.getId())
                .toolId(tool == null ? null : tool.getId())
                .sideEffectLevel(tool == null ? McpSideEffectLevel.READ_ONLY : tool.getSideEffectLevel())
                .status(status)
                .policyDecision(policyDecision)
                .redactionStatus(redactionStatus)
                .inputHash(inputHash)
                .outputHash(outputHash)
                .inputSummary(summary(redactedInput.getValue()))
                .outputSummary(summary(redactedOutput.getValue()))
                .startedAt(started)
                .completedAt(completed)
                .durationMillis(Duration.between(started, completed).toMillis())
                .sourceRefs(sourceRefs)
                .diagnostics(diagnostics == null ? new ArrayList<>() : new ArrayList<>(diagnostics))
                .build();
        store().save(receipt);
        AgentToolCall toolCall = AgentToolCall.builder()
                .id("tool-call-" + java.util.UUID.nameUUIDFromBytes(receiptId.getBytes(java.nio.charset.StandardCharsets.UTF_8)))
                .toolName(tool == null ? null : tool.getId())
                .serverId(server == null ? null : server.getId())
                .sideEffectLevel(tool == null || tool.getSideEffectLevel() == null ? null : tool.getSideEffectLevel().name())
                .status(status.name())
                .startedAt(started)
                .completedAt(completed)
                .arguments(request == null || request.getArguments() == null ? new LinkedHashMap<>() : new LinkedHashMap<>(request.getArguments()))
                .resultSummary(receipt.getOutputSummary())
                .evidenceReceiptId(receipt.getId())
                .inputHash(inputHash)
                .outputHash(outputHash)
                .redactionStatus(redactionStatus)
                .policyDecision(policyDecision)
                .durationMillis(receipt.getDurationMillis())
                .sourceRefs(sourceRefs)
                .diagnostics(receipt.getDiagnostics())
                .build();
        return McpToolInvocationResult.builder()
                .id("mcp-result-" + java.util.UUID.nameUUIDFromBytes(receiptId.getBytes(java.nio.charset.StandardCharsets.UTF_8)))
                .serverId(server == null ? null : server.getId())
                .toolId(tool == null ? null : tool.getId())
                .status(status)
                .policyDecision(policyDecision)
                .startedAt(started)
                .completedAt(completed)
                .result(result)
                .resultSummary(receipt.getOutputSummary())
                .evidenceReceipt(receipt)
                .toolCall(toolCall)
                .sourceRefs(sourceRefs)
                .diagnostics(receipt.getDiagnostics())
                .build();
    }

    private List<String> missingArguments(McpToolDescriptor descriptor, Map<String, Object> arguments) {
        if (descriptor == null || descriptor.getRequiredArguments() == null || descriptor.getRequiredArguments().isEmpty()) {
            return Collections.emptyList();
        }
        List<String> missing = new ArrayList<>();
        for (String required : descriptor.getRequiredArguments()) {
            Object value = arguments == null ? null : arguments.get(required);
            if (value == null || String.valueOf(value).trim().isEmpty()) {
                missing.add(required);
            }
        }
        return missing;
    }

    private List<AgentSourceCitation> sourceRefs(Object result, McpToolDescriptor tool) {
        List<AgentSourceCitation> refs = new ArrayList<>();
        if (tool != null && tool.getSourceFamilies() != null) {
            for (String family : tool.getSourceFamilies()) {
                refs.add(citation(family, tool.getId(), "MCP tool " + tool.getId()));
            }
        }
        collectSourceRefs(result, refs);
        if (refs.isEmpty() && tool != null) {
            refs.add(citation("MCP", tool.getId(), "MCP tool receipt"));
        }
        return refs;
    }

    @SuppressWarnings("unchecked")
    private void collectSourceRefs(Object value, List<AgentSourceCitation> refs) {
        if (value == null) {
            return;
        }
        Object normalized;
        try {
            normalized = new com.fasterxml.jackson.databind.ObjectMapper().convertValue(value, Object.class);
        } catch (IllegalArgumentException ex) {
            return;
        }
        collectFromNormalized(normalized, refs);
    }

    @SuppressWarnings("unchecked")
    private void collectFromNormalized(Object value, List<AgentSourceCitation> refs) {
        if (value instanceof Map<?, ?>) {
            Map<?, ?> map = (Map<?, ?>) value;
            Object sourceRefs = map.get("sourceRefs");
            if (sourceRefs instanceof List<?>) {
                List<?> list = (List<?>) sourceRefs;
                for (Object ref : list) {
                    refs.add(citation(String.valueOf(ref)));
                }
            }
            Object id = map.get("id");
            if (id != null) {
                Object family = map.get("family");
                refs.add(citation(family == null ? "SOURCE" : String.valueOf(family), String.valueOf(id), "Tool result object"));
            }
            for (Object child : map.values()) {
                collectFromNormalized(child, refs);
            }
        } else if (value instanceof List<?>) {
            List<?> list = (List<?>) value;
            for (Object item : list) {
                collectFromNormalized(item, refs);
            }
        }
    }

    private AgentSourceCitation citation(String ref) {
        String safe = ref == null ? "unknown" : ref;
        String family = safe.contains(":") ? safe.substring(0, safe.indexOf(':')) : "SOURCE";
        String id = safe.contains(":") ? safe.substring(safe.indexOf(':') + 1) : safe;
        return citation(family, id, safe);
    }

    private AgentSourceCitation citation(String family, String id, String label) {
        String normalizedFamily = family == null ? "SOURCE" : family.toUpperCase(Locale.US);
        return AgentSourceCitation.builder()
                .sourceFamily(normalizedFamily)
                .sourceId(id)
                .label(label)
                .route(routeFor(normalizedFamily, id))
                .build();
    }

    private String routeFor(String family, String id) {
        if (family == null || id == null) {
            return null;
        }
        if (family.contains("MISSION")) return "/missions/" + id;
        if (family.contains("RESERVATION")) return "/missions/" + id + "/reservations/" + id;
        if (family.contains("DECISION") || family.contains("AUDIT")) return "/decisions/" + id;
        if (family.contains("FEED") || family.contains("USNS")) return "/feed/" + id;
        if (family.contains("REFERENCE")) return "/config";
        if (family.contains("WEATHER") || family.contains("PIREP") || family.contains("NOTAM")) return "/weather";
        return null;
    }

    private McpReceiptStore store() {
        return receiptStore == null ? new McpReceiptStore() : receiptStore;
    }

    private String summary(Object value) {
        String json = CanonicalJson.write(value);
        return json.length() > 700 ? json.substring(0, 700) + "...[TRUNCATED]" : json;
    }

    private String required(Map<String, Object> arguments, String key) {
        String value = string(arguments, key);
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException("Missing required argument: " + key);
        }
        return value;
    }

    private String string(Map<String, Object> arguments, String key) {
        Object value = arguments == null ? null : arguments.get(key);
        return value == null ? null : String.valueOf(value);
    }

    private Integer integer(Map<String, Object> arguments, String key) {
        return integer(arguments, key, null);
    }

    private Integer integer(Map<String, Object> arguments, String key, Integer fallback) {
        Object value = arguments == null ? null : arguments.get(key);
        if (value == null || String.valueOf(value).trim().isEmpty()) {
            return fallback;
        }
        try {
            return value instanceof Number ? ((Number) value).intValue() : Integer.parseInt(String.valueOf(value));
        } catch (NumberFormatException ex) {
            return fallback;
        }
    }

    private Double number(Map<String, Object> arguments, String key, Double fallback) {
        Object value = arguments == null ? null : arguments.get(key);
        if (value == null || String.valueOf(value).trim().isEmpty()) {
            return fallback;
        }
        try {
            return value instanceof Number ? ((Number) value).doubleValue() : Double.parseDouble(String.valueOf(value));
        } catch (NumberFormatException ex) {
            return fallback;
        }
    }

    private String value(String value) {
        return value == null ? "" : value;
    }
}
