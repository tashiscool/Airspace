package org.tash.extensions.agentic;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.InMemoryReservationWorkflowRepository;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

class AgentAdversarialCorpusRunner {
    private final ObjectMapper mapper = new ObjectMapper();

    List<AgentAdversarialScenario> load() {
        try (InputStream input = getClass().getResourceAsStream("/agentic/adversarial/corpus.json")) {
            if (input == null) {
                throw new IllegalStateException("Missing adversarial corpus resource");
            }
            return mapper.readValue(input, new TypeReference<List<AgentAdversarialScenario>>() {});
        } catch (IOException ex) {
            throw new IllegalStateException("Unable to load adversarial corpus", ex);
        }
    }

    AgentRunResult run(AgentAdversarialScenario scenario) {
        AirspaceProductService productService = seededProductService();
        String missionId = productService.missions().get(0).getId();
        String reservationId = productService.mission(missionId).getReservations().get(0).getId();
        for (AgentAdversarialScenario.MessageFixture message : scenario.messages) {
            ProductDtos.MessageRequest request = new ProductDtos.MessageRequest();
            request.setMissionId(missionId);
            request.setReservationId(reservationId);
            request.setFamily(message.family);
            request.setDirection("INBOUND");
            request.setSubject(message.subject);
            request.setRawText(message.rawText);
            productService.sendMessage(request);
        }
        for (AgentAdversarialScenario.FeedFixture feed : scenario.feeds) {
            ProductDtos.FeedIngestRequest request = new ProductDtos.FeedIngestRequest();
            request.setSourceId(feed.sourceId);
            request.setType(feed.type);
            request.setRawPayload(envelope(feed.body));
            productService.ingestFeed(request);
        }
        AgenticOperationsService service = wiredService(productService);
        AgentRunRequest request = new AgentRunRequest();
        request.setAgentType("DATA_INTEGRITY");
        request.setMissionId(missionId);
        request.setReservationId(reservationId);
        request.setActor("adversarial-corpus");
        return service.dataIntegrity(request);
    }

    private AirspaceProductService seededProductService() {
        AirspaceProductService service = new AirspaceProductService(new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber("ADV-1");
        missionRequest.setTitle("Agent adversarial corpus mission");
        ProductDtos.MissionSummary mission = service.createMission(missionRequest);
        ProductDtos.ReservationRequest reservationRequest = new ProductDtos.ReservationRequest();
        reservationRequest.setActor("planner");
        reservationRequest.setRawText("A. ADV-1\nB. 1KC135/M\nC. JFK 200000Z\nD. 3000N15000W 3100N14900W\nE. JFK\nF. FL240-FL280\nG. ADVERSARIAL TEST");
        String reservationId = service.createReservation(mission.getId(), reservationRequest).getRecord().getId();
        ProductDtos.MessageRequest sigmet = new ProductDtos.MessageRequest();
        sigmet.setMissionId(mission.getId());
        sigmet.setReservationId(reservationId);
        sigmet.setFamily("SIGMET");
        sigmet.setDirection("INBOUND");
        sigmet.setSubject("Baseline severe SIGMET");
        sigmet.setRawText("SIGMET ADV VALID 200000/200400 FROM 3000N15000W TO 3100N14900W EMBD TS MOV E 25KT TOP FL450 INTSF");
        service.sendMessage(sigmet);
        ProductDtos.MessageRequest notam = new ProductDtos.MessageRequest();
        notam.setMissionId(mission.getId());
        notam.setReservationId(reservationId);
        notam.setFamily("FDC");
        notam.setDirection("INBOUND");
        notam.setSubject("Baseline FDC NOTAM");
        notam.setRawText("!FDC ADV NOTAM AIRSPACE CLSD WI AN AREA 3000N15000W-3100N14900W SFC-FL260");
        service.sendMessage(notam);
        return service;
    }

    private AgenticOperationsService wiredService(AirspaceProductService productService) {
        AgenticOperationsService service = new AgenticOperationsService();
        service.dataIntegrityAgent = new DataIntegrityAgent();
        service.dataIntegrityAgent.productService = productService;
        service.citationValidator = new AgentCitationValidator();
        service.policyEnforcer = new AgentPolicyEnforcer();
        service.evaluationService = new AgentEvaluationService();
        service.evaluationService.citationValidator = service.citationValidator;
        service.evaluationService.policyEnforcer = service.policyEnforcer;
        service.auditService = new AgentAuditService();
        return service;
    }

    private String envelope(String body) {
        return "01GGNC07GP\n"
                + "CNS000 300334\n"
                + "GG KDZZNAXX\n"
                + "300334 KGPS\n"
                + MessageControlCharacters.STX + body
                + MessageControlCharacters.VT + MessageControlCharacters.ETX;
    }
}
