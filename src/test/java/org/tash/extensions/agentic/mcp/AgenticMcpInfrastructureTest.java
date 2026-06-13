package org.tash.extensions.agentic.mcp;

import org.junit.jupiter.api.Test;
import org.tash.extensions.agentic.AgentPolicy;
import org.tash.extensions.agentic.AgentPolicyEnforcer;
import org.tash.extensions.agentic.AgenticRiskAssessment;
import org.tash.extensions.agentic.AgenticRiskChecklistService;
import org.tash.extensions.agentic.AgentRunResult;
import org.tash.extensions.agentic.AgentToolCall;
import org.tash.extensions.agentic.HumanReviewMode;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.InMemoryReservationWorkflowRepository;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.util.LinkedHashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class AgenticMcpInfrastructureTest {
    @Test
    void registryListsCuratedFirstPartyAndSetupRequiredExternalTools() {
        CuratedMcpCatalog catalog = new CuratedMcpCatalog();

        assertTrue(catalog.server(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY).orElseThrow().isEnabled());
        assertFalse(catalog.server(CuratedMcpCatalog.EXTERNAL_MCP).orElseThrow().isEnabled());
        assertTrue(catalog.tools(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY).stream()
                .anyMatch(tool -> "airspace.mission.weather_verdict".equals(tool.getId())
                        && tool.getSideEffectLevel() == McpSideEffectLevel.READ_ONLY));
        assertTrue(catalog.tools(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY).stream()
                .anyMatch(tool -> "airspace.coordination.draft".equals(tool.getId())
                        && tool.getSideEffectLevel() == McpSideEffectLevel.DRAFT_ONLY));
        assertTrue(catalog.tools(CuratedMcpCatalog.EXTERNAL_MCP).get(0).isSetupRequired());
        assertTrue(catalog.tools(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY).stream()
                .allMatch(tool -> tool.getRiskProfile() != null
                        && tool.getRiskProfile().getWorstCaseBlastRadius() != null
                        && tool.getRiskProfile().getDataEgress() != null));
        assertEquals(HumanReviewMode.PUSH_APPROVAL, catalog.tool(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY, "airspace.coordination.draft")
                .orElseThrow().getRiskProfile().getRequiredHumanReviewMode());
        assertEquals(HumanReviewMode.PUSH_APPROVAL, catalog.tools(CuratedMcpCatalog.EXTERNAL_MCP).get(0)
                .getRiskProfile().getRequiredHumanReviewMode());
    }

    @Test
    void riskChecklistReportsBlastRadiusEgressAndSetupRequiredDiagnostics() {
        AgenticRiskChecklistService service = new AgenticRiskChecklistService(new CuratedMcpCatalog());

        java.util.List<AgenticRiskAssessment> assessments = service.assessments();

        assertFalse(assessments.isEmpty());
        assertTrue(assessments.stream().anyMatch(assessment -> "airspace.coordination.draft".equals(assessment.getSubjectId())
                && assessment.getRiskProfile().getRequiredHumanReviewMode() == HumanReviewMode.PUSH_APPROVAL
                && assessment.getRiskProfile().getWorstCaseBlastRadius().contains("draft")));
        assertTrue(assessments.stream().anyMatch(assessment -> "external-mcp.configured_tool".equals(assessment.getSubjectId())
                && !assessment.getDiagnostics().isEmpty()));
    }

    @Test
    void firstPartyInvocationReturnsReceiptAndUnknownOrExternalToolsAreBlocked() {
        AirspaceProductService productService = seededProductService();
        CuratedMcpInvocationService service = invocationService(productService);
        service.productService = productService;
        String missionId = productService.missions().get(0).getId();

        Map<String, Object> args = new LinkedHashMap<>();
        args.put("missionId", missionId);
        McpToolInvocationResult accepted = service.call(McpToolInvocationRequest.builder()
                .serverId(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY)
                .toolId("airspace.mission.weather_verdict")
                .arguments(args)
                .build());

        assertEquals(McpInvocationStatus.ACCEPTED, accepted.getStatus(), accepted.getDiagnostics().toString());
        assertNotNull(accepted.getEvidenceReceipt());
        assertEquals("CLEAN", accepted.getEvidenceReceipt().getRedactionStatus());
        assertNotNull(accepted.getEvidenceReceipt().getInputHash());
        assertNotNull(accepted.getEvidenceReceipt().getOutputHash());
        assertFalse(accepted.getToolCall().getSourceRefs().isEmpty());
        assertEquals(1, service.receipts(10).size());

        McpToolInvocationResult unknown = service.call(McpToolInvocationRequest.builder()
                .serverId(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY)
                .toolId("airspace.unknown")
                .arguments(args)
                .build());
        assertEquals(McpInvocationStatus.DENIED, unknown.getStatus());

        McpToolInvocationResult external = service.call(McpToolInvocationRequest.builder()
                .serverId(CuratedMcpCatalog.EXTERNAL_MCP)
                .toolId("external-mcp.configured_tool")
                .arguments(args)
                .build());
        assertEquals(McpInvocationStatus.SETUP_REQUIRED, external.getStatus());
    }

    @Test
    void redactorRemovesSensitiveFieldsAndPolicyBlocksMutatingToolCalls() {
        McpRedactor redactor = new McpRedactor();
        Map<String, Object> value = new LinkedHashMap<>();
        value.put("apiKey", "secret-value");
        value.put("rawPayload", "raw text");
        value.put("missionId", "mission-1");

        McpRedactionResult result = redactor.redact(value);

        assertEquals("REDACTED", result.getStatus());
        assertTrue(String.valueOf(result.getValue()).contains("[REDACTED]"));
        assertFalse(String.valueOf(result.getValue()).contains("secret-value"));

        AgentPolicyEnforcer enforcer = new AgentPolicyEnforcer();
        AgentRunResult run = AgentRunResult.builder()
                .id("agent-mutating-tool")
                .agentType("MISSION_RISK")
                .accepted(true)
                .toolCalls(java.util.Collections.singletonList(AgentToolCall.builder()
                        .id("tool-mutating")
                        .toolName("airspace.reservation.approve")
                        .sideEffectLevel("MUTATING")
                        .build()))
                .build();

        assertFalse(enforcer.validate(run, AgentPolicy.builder().build()).isEmpty());
    }

    @Test
    void receiptHashesAreDeterministicForSameInputsAndOutputs() {
        AirspaceProductService productService = seededProductService();
        CuratedMcpInvocationService service = invocationService(productService);
        String missionId = productService.missions().get(0).getId();
        Map<String, Object> args = new LinkedHashMap<>();
        args.put("missionId", missionId);

        McpToolInvocationRequest request = McpToolInvocationRequest.builder()
                .serverId(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY)
                .toolId("airspace.mission.weather_verdict")
                .arguments(args)
                .build();

        McpToolInvocationResult first = service.call(request);
        McpToolInvocationResult second = service.call(request);

        assertEquals(first.getEvidenceReceipt().getInputHash(), second.getEvidenceReceipt().getInputHash());
        assertEquals(first.getEvidenceReceipt().getOutputHash(), second.getEvidenceReceipt().getOutputHash());
        assertEquals(first.getEvidenceReceipt().getId(), second.getEvidenceReceipt().getId());
    }

    private CuratedMcpInvocationService invocationService(AirspaceProductService productService) {
        CuratedMcpInvocationService service = new CuratedMcpInvocationService();
        service.catalog = new CuratedMcpCatalog();
        service.policyGate = new McpPolicyGate();
        service.redactor = new McpRedactor();
        service.receiptStore = new McpReceiptStore();
        service.productService = productService;
        return service;
    }

    private AirspaceProductService seededProductService() {
        AirspaceProductService service = new AirspaceProductService(new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber("MCP-1");
        missionRequest.setTitle("MCP agent tool mission");
        ProductDtos.MissionSummary mission = service.createMission(missionRequest);

        ProductDtos.ReservationRequest reservationRequest = new ProductDtos.ReservationRequest();
        reservationRequest.setActor("planner");
        reservationRequest.setRawText("A. MCP-1\nB. 1KC135/M\nC. JFK 200000Z\nD. 3000N15000W 3100N14900W\nE. JFK\nF. FL240-FL280\nG. MCP TEST");
        String reservationId = service.createReservation(mission.getId(), reservationRequest).getRecord().getId();

        ProductDtos.MessageRequest sigmet = new ProductDtos.MessageRequest();
        sigmet.setMissionId(mission.getId());
        sigmet.setReservationId(reservationId);
        sigmet.setFamily("SIGMET");
        sigmet.setDirection("INBOUND");
        sigmet.setSubject("MCP SIGMET");
        sigmet.setRawText("SIGMET MCP VALID 200000/200400 FROM 3000N15000W TO 3100N14900W EMBD TS MOV E 25KT TOP FL450 INTSF");
        service.sendMessage(sigmet);

        ProductDtos.FeedIngestRequest feed = new ProductDtos.FeedIngestRequest();
        feed.setSourceId("mcp-usns");
        feed.setType("USNS");
        feed.setRawPayload("01GGNC07GP\nCNS000 300334\nGG KDZZNAXX\n300334 KGPS\n"
                + MessageControlCharacters.STX
                + "SIGMET MCP VALID 200000/200400 FROM 3000N15000W TO 3100N14900W EMBD TS MOV E 25KT"
                + MessageControlCharacters.VT + MessageControlCharacters.ETX);
        service.ingestFeed(feed);
        return service;
    }
}
