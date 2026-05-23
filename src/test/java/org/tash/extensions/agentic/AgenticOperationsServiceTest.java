package org.tash.extensions.agentic;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.InMemoryReservationWorkflowRepository;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.nio.file.Path;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;

class AgenticOperationsServiceTest {
    @Test
    void agenticOperationsProducesCitedFindingsDraftsAndAudit() {
        AirspaceProductService productService = seededProductService();
        AgenticOperationsService service = wiredService(productService);
        String missionId = productService.missions().get(0).getId();
        String reservationId = productService.mission(missionId).getReservations().get(0).getId();

        AgentRunRequest request = new AgentRunRequest();
        request.setAgentType("ALL");
        request.setMissionId(missionId);
        request.setReservationId(reservationId);
        request.setActor("planner");
        AgentRunResult result = service.run(request);

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertEquals("ALL", result.getAgentType());
        assertNotNull(result.getAuditEnvelope());
        assertNotNull(result.getEvaluation());
        assertNotNull(result.getReasoningEnvelope());
        assertEquals("agentic-nextgen-v1", result.getReasoningEnvelope().getPromptVersion());
        assertEquals("deterministic-draft-only", result.getReasoningEnvelope().getModelId());
        assertEquals("DETERMINISTIC_DRAFT_ONLY", result.getReasoningEnvelope().getReasoningMode());
        assertNotNull(result.getReasoningEnvelope().getDraftHash());
        assertTrue(result.getReasoningEnvelope().getInputSummary().contains("Agent ALL"));
        assertTrue(result.getReasoningEnvelope().getRequiredOutputRules().stream()
                .anyMatch(rule -> rule.contains("Every operational claim must cite")));
        assertTrue(result.getReasoningEnvelope().getProhibitedActions().stream()
                .anyMatch(action -> action.contains("External sends prohibited")));
        assertTrue(result.getReasoningEnvelope().getProhibitedActions().stream()
                .anyMatch(action -> action.contains("Official workflow mutation prohibited")));
        assertTrue(result.getEvaluation().isAccepted(), result.getEvaluation().getErrors().toString());
        assertEquals(0, result.getEvaluation().getUncitedClaimCount());
        assertTrue(result.getEvaluation().getCitationCoverage() >= 0.99);
        assertFalse(result.getOperatingLoop().isEmpty());
        assertFalse(result.getDeltas().isEmpty());
        assertTrue(result.getOperatingLoop().stream().anyMatch(step -> "OBSERVE".equals(step.getStage())));
        assertTrue(result.getOperatingLoop().stream().anyMatch(step -> "AUDIT".equals(step.getStage())));
        assertNotNull(result.getAuditEnvelope().getInputHash());
        assertNotNull(result.getAuditEnvelope().getOutputHash());
        assertFalse(result.getFindings().isEmpty());
        assertFalse(result.getRecommendations().isEmpty());
        assertFalse(result.getTasks().isEmpty());
        assertTrue(result.getRecommendations().stream().anyMatch(AgentRecommendation::isHumanApprovalRequired));
        assertTrue(result.getFindings().stream().allMatch(finding -> finding.getCitations() != null && !finding.getCitations().isEmpty()));
        assertFalse(service.runs(10).isEmpty());
        assertFalse(service.tasks(null, 10).isEmpty());
        assertTrue(service.metrics().get("agentic.runs") >= 1.0);
        assertTrue(service.metrics().get("agentic.runs.all") >= 1.0);
        assertTrue(service.metrics().get("agentic.tasks") >= 1.0);
        assertEquals(0.0, service.metrics().get("agentic.store.durable"));
        AgentHistoryQuery contextQuery = new AgentHistoryQuery();
        contextQuery.setMissionId(missionId);
        contextQuery.setReservationId(reservationId);
        contextQuery.setAgentType("ALL");
        assertEquals(result.getId(), service.runs(contextQuery).get(0).getId());

        AgentTask firstTask = service.tasks(null, 10).get(0);
        AgentTaskTransitionRequest transition = new AgentTaskTransitionRequest();
        transition.setStatus("ACKNOWLEDGED");
        transition.setActor("planner");
        transition.setNote("reviewed in unit test");
        AgentTask acknowledged = service.transitionTask(firstTask.getId(), transition);
        assertEquals("ACKNOWLEDGED", acknowledged.getStatus());
        assertTrue(acknowledged.getRationale().contains("reviewed in unit test"));
    }

    @Test
    void inMemoryAgentRunStoreListsFiltersAndTransitionsTasks() {
        InMemoryAgentRunStore store = new InMemoryAgentRunStore();
        AgentRunResult run = AgentRunResult.builder()
                .id("run-1")
                .agentType("MISSION_RISK")
                .missionId("mission-1")
                .decisionId("decision-1")
                .accepted(true)
                .citations(java.util.Collections.singletonList(AgentSourceCitation.builder()
                        .sourceFamily("WEATHER")
                        .sourceId("wx-1")
                        .build()))
                .build();
        AgentTask task = AgentTask.builder()
                .id("task-1")
                .title("Review")
                .status("OPEN")
                .priority("HIGH")
                .assignedRole("planner")
                .route("/missions/mission-1")
                .rationale("Needs review")
                .citations(java.util.Collections.singletonList(AgentSourceCitation.builder()
                        .sourceFamily("WEATHER")
                        .sourceId("wx-1")
                        .build()))
                .build();

        store.saveRun(run);
        store.saveTask(task);

        assertEquals("run-1", store.findRun("run-1").orElseThrow().getId());
        assertEquals(1, store.runs(10).size());
        assertEquals(1, store.tasks("OPEN", 10).size());
        assertEquals(0, store.tasks("ACKNOWLEDGED", 10).size());
        assertEquals("IN_MEMORY", store.status().getMode());
        assertFalse(store.status().isDurable());
        assertEquals(1, store.status().getRunCount());
        assertEquals(1, store.status().getTaskCount());
        AgentHistoryQuery runQuery = new AgentHistoryQuery();
        runQuery.setMissionId("mission-1");
        runQuery.setDecisionId("decision-1");
        runQuery.setAgentType("MISSION_RISK");
        runQuery.setAccepted(true);
        runQuery.setSourceFamily("WEATHER");
        assertEquals(1, store.runs(runQuery).size());
        runQuery.setSourceFamily("NOTAM");
        assertEquals(0, store.runs(runQuery).size());

        AgentHistoryQuery taskQuery = new AgentHistoryQuery();
        taskQuery.setTaskStatus("OPEN");
        taskQuery.setTaskPriority("HIGH");
        taskQuery.setAssignedRole("planner");
        taskQuery.setRouteContains("mission-1");
        taskQuery.setSourceFamily("WEATHER");
        assertEquals(1, store.tasks(taskQuery).size());

        AgentTaskTransitionRequest transition = new AgentTaskTransitionRequest();
        transition.setStatus("ACKNOWLEDGED");
        transition.setActor("planner");
        transition.setNote("handled");
        AgentTask updated = store.transitionTask("task-1", transition);

        assertEquals("ACKNOWLEDGED", updated.getStatus());
        assertTrue(updated.getRationale().contains("handled"));
        assertEquals(1, store.tasks("ACKNOWLEDGED", 10).size());
    }

    @Test
    void jsonFileAgentRunStorePersistsRunsTasksAndTransitions(@TempDir Path tempDir) {
        Path file = tempDir.resolve("agent-runs.json");
        JsonFileAgentRunStore store = new JsonFileAgentRunStore(file);
        AgentRunResult run = AgentRunResult.builder()
                .id("run-json")
                .agentType("REPLAY_AUDIT")
                .missionId("mission-json")
                .decisionId("decision-json")
                .accepted(true)
                .traceAnswer(AgentTraceAnswer.builder()
                        .question("Why?")
                        .answer("Because cited trace")
                        .citations(java.util.Collections.singletonList(AgentSourceCitation.builder()
                                .sourceFamily("DECISION")
                                .sourceId("decision-json")
                                .build()))
                        .build())
                .citations(java.util.Collections.singletonList(AgentSourceCitation.builder()
                        .sourceFamily("DECISION")
                        .sourceId("decision-json")
                        .build()))
                .build();
        AgentTask task = AgentTask.builder()
                .id("task-json")
                .title("Replay review")
                .status("OPEN")
                .priority("MEDIUM")
                .assignedRole("supervisor")
                .route("/decisions/decision-json")
                .rationale("Review trace answer")
                .citations(run.getCitations())
                .build();

        store.saveRun(run);
        store.saveTask(task);
        AgentTaskTransitionRequest transition = new AgentTaskTransitionRequest();
        transition.setStatus("ACKNOWLEDGED");
        transition.setActor("supervisor");
        transition.setNote("persisted transition");
        store.transitionTask("task-json", transition);

        JsonFileAgentRunStore reloaded = new JsonFileAgentRunStore(file);
        assertEquals("JSON_FILE", reloaded.status().getMode());
        assertTrue(reloaded.status().isDurable());
        assertEquals(file.toString(), reloaded.status().getPath());
        assertEquals(1, reloaded.status().getRunCount());
        assertEquals(1, reloaded.status().getTaskCount());
        AgentHistoryQuery runQuery = new AgentHistoryQuery();
        runQuery.setMissionId("mission-json");
        runQuery.setSourceFamily("DECISION");
        assertEquals("run-json", reloaded.runs(runQuery).get(0).getId());
        assertEquals("Why?", reloaded.findRun("run-json").orElseThrow().getTraceAnswer().getQuestion());

        AgentHistoryQuery taskQuery = new AgentHistoryQuery();
        taskQuery.setTaskStatus("ACKNOWLEDGED");
        taskQuery.setAssignedRole("supervisor");
        taskQuery.setRouteContains("decision-json");
        assertEquals("task-json", reloaded.tasks(taskQuery).get(0).getId());
        assertTrue(reloaded.findTask("task-json").orElseThrow().getRationale().contains("persisted transition"));
    }

    @Test
    void agentRunStoreProducerSelectsJsonStoreWhenPathConfigured(@TempDir Path tempDir) {
        AgentRunStoreProducer producer = new AgentRunStoreProducer();

        assertInstanceOf(InMemoryAgentRunStore.class, producer.agentRunStore(Optional.empty()));
        assertInstanceOf(JsonFileAgentRunStore.class, producer.agentRunStore(Optional.of(tempDir.resolve("agent-store.json").toString())));
    }

    @Test
    void citationValidatorBlocksUnsupportedOperationalClaims() {
        AgentCitationValidator validator = new AgentCitationValidator();
        AgentRunResult result = AgentRunResult.builder()
                .id("agent-test")
                .agentType("MISSION_RISK")
                .accepted(true)
                .findings(java.util.Collections.singletonList(AgentFinding.builder()
                        .id("finding-test")
                        .category("MISSION_RISK")
                        .severity("HIGH")
                        .message("Unsupported operational claim")
                        .confidence(0.8)
                        .build()))
                .build();

        assertFalse(validator.validate(result, AgentPolicy.builder().build()).isEmpty());
    }

    @Test
    void specificAgentsKeepOfficialActionsHumanApproved() {
        AirspaceProductService productService = seededProductService();
        AgenticOperationsService service = wiredService(productService);
        String missionId = productService.missions().get(0).getId();

        AgentRunRequest request = new AgentRunRequest();
        request.setMissionId(missionId);
        AgentRunResult coordination = service.coordinationDraft(request);
        AgentRunResult reroute = service.rerouteAnalysis(request);
        request.setQuestion("Why this reroute?");
        AgentRunResult replay = service.replayAudit(request);

        assertTrue(coordination.getRecommendations().stream().allMatch(AgentRecommendation::isHumanApprovalRequired));
        assertTrue(reroute.getRecommendations().stream().allMatch(AgentRecommendation::isHumanApprovalRequired));
        assertTrue(coordination.getOperatingLoop().stream().anyMatch(step -> "COORDINATE".equals(step.getStage())));
        assertTrue(coordination.getSummary().contains("send is blocked"));
        assertFalse(reroute.getFindings().isEmpty());
        assertNotNull(replay.getTraceAnswer());
        assertEquals("Why this reroute?", replay.getTraceAnswer().getQuestion());
        assertTrue(replay.getTraceAnswer().getAnswer().contains("Source refs"));
        assertFalse(replay.getTraceAnswer().getCitations().isEmpty());
    }

    @Test
    void operationalDeltaServiceComparesDecisionAndRouteImpactChanges() {
        AirspaceProductService productService = seededProductService();
        String missionId = productService.missions().get(0).getId();
        String reservationId = productService.mission(missionId).getReservations().get(0).getId();
        ProductDtos.DecisionEvaluateRequest request = decisionRequest(missionId, reservationId);
        ProductDtos.DecisionSummary previous = productService.evaluateDecision(request);

        ProductDtos.MessageRequest pirep = new ProductDtos.MessageRequest();
        pirep.setMissionId(missionId);
        pirep.setReservationId(reservationId);
        pirep.setFamily("PIREP");
        pirep.setDirection("INBOUND");
        pirep.setSubject("Severe route PIREP");
        pirep.setRawText("UUA /OV 3000N15000W/TM 2005/FL250/TP B738/TB SEV/RM URGENT");
        productService.sendMessage(pirep);
        ProductDtos.DecisionSummary current = productService.evaluateDecision(request);

        OperationalDeltaService deltaService = new OperationalDeltaService();
        deltaService.productService = productService;
        java.util.List<AgentOperationalDelta> deltas = deltaService.compare(previous.getId(), current.getId(), missionId);

        assertFalse(deltas.isEmpty());
        assertTrue(deltas.stream().anyMatch(delta -> java.util.Arrays.asList(
                "SOURCE_ADDED", "CONFIDENCE_CHANGED", "ACTION_CHANGED", "ROUTE_CANDIDATES_AVAILABLE").contains(delta.getChangeType())));
        assertTrue(deltas.stream().allMatch(delta -> delta.getCitations() != null && !delta.getCitations().isEmpty()));
    }

    @Test
    void configurableLlmProviderIsDisabledByDefaultAndLocalModeIsCited() {
        AgentRunResult draft = AgentRunResult.builder()
                .id("draft-1")
                .agentType("MISSION_RISK")
                .accepted(true)
                .summary("draft")
                .reasoningEnvelope(AgentReasoningEnvelope.builder()
                        .modelId("deterministic-draft-only")
                        .reasoningMode("DETERMINISTIC_DRAFT_ONLY")
                        .build())
                .citations(java.util.Collections.singletonList(AgentSourceCitation.builder()
                        .sourceFamily("WEATHER")
                        .sourceId("wx-1")
                        .build()))
                .build();
        ConfigurableLlmReasoningProvider provider = new ConfigurableLlmReasoningProvider();
        provider.mode = "disabled";
        provider.endpoint = Optional.empty();
        provider.apiKey = Optional.empty();
        provider.model = "none";
        provider.timeoutMillis = 250;
        assertSame(draft, provider.reason(new AgentRunRequest(), draft));

        provider.mode = "local-test";
        provider.model = "local-test-model";
        AgentRunResult reasoned = provider.reason(new AgentRunRequest(), draft);
        assertNotSame(draft, reasoned);
        assertEquals("LOCAL_TEST_PROVIDER", reasoned.getReasoningEnvelope().getReasoningMode());
        assertTrue(reasoned.getFindings().stream().anyMatch(finding -> "LLM_REASONING".equals(finding.getCategory())));
        assertTrue(reasoned.getFindings().stream().allMatch(finding -> !finding.getCitations().isEmpty()));
    }

    @Test
    void dataIntegrityDetectsContradictionsResidualRiskAndPirepRelevance() {
        AirspaceProductService productService = seededProductService();
        String missionId = productService.missions().get(0).getId();
        String reservationId = productService.mission(missionId).getReservations().get(0).getId();
        ProductDtos.MessageRequest smoothPirep = new ProductDtos.MessageRequest();
        smoothPirep.setMissionId(missionId);
        smoothPirep.setReservationId(reservationId);
        smoothPirep.setFamily("PIREP");
        smoothPirep.setDirection("INBOUND");
        smoothPirep.setSubject("Contradictory smooth PIREP");
        smoothPirep.setRawText("UA /OV 7000N10000W/TM 2000/FL120/TP B738/TB SMOOTH/IC NEG/RM FAR OFF ROUTE");
        productService.sendMessage(smoothPirep);
        ProductDtos.MessageRequest ambiguousDomNotam = new ProductDtos.MessageRequest();
        ambiguousDomNotam.setMissionId(missionId);
        ambiguousDomNotam.setReservationId(reservationId);
        ambiguousDomNotam.setFamily("DOMESTIC");
        ambiguousDomNotam.setDirection("INBOUND");
        ambiguousDomNotam.setSubject("Ambiguous domestic NOTAM");
        ambiguousDomNotam.setRawText("!DCA LDN RAMP UNUSUAL LEGACY TEXT 1012211200-1012211300");
        productService.sendMessage(ambiguousDomNotam);
        ProductDtos.MessageRequest nonGeometricIcaoNotam = new ProductDtos.MessageRequest();
        nonGeometricIcaoNotam.setMissionId(missionId);
        nonGeometricIcaoNotam.setReservationId(reservationId);
        nonGeometricIcaoNotam.setFamily("ICAO_NOTAMC");
        nonGeometricIcaoNotam.setDirection("INBOUND");
        nonGeometricIcaoNotam.setSubject("Cancellation without geometry");
        nonGeometricIcaoNotam.setRawText("(A0002/26 NOTAMC A) KZNY E) CANCEL TEST)");
        productService.sendMessage(nonGeometricIcaoNotam);
        ProductDtos.MessageRequest emptyFirIcaoNotam = new ProductDtos.MessageRequest();
        emptyFirIcaoNotam.setMissionId(missionId);
        emptyFirIcaoNotam.setReservationId(reservationId);
        emptyFirIcaoNotam.setFamily("ICAO_NOTAMN");
        emptyFirIcaoNotam.setDirection("INBOUND");
        emptyFirIcaoNotam.setSubject("Empty FIR Q-line");
        emptyFirIcaoNotam.setRawText("(A0001/26 NOTAMN Q) /QRTCA/IV/BO/W/000/180/3000N15000W005 A) KZNY B) 2601011200 C) PERM E) TEST)");
        productService.sendMessage(emptyFirIcaoNotam);
        ProductDtos.MessageRequest malformedAltrv = new ProductDtos.MessageRequest();
        malformedAltrv.setMissionId(missionId);
        malformedAltrv.setReservationId(reservationId);
        malformedAltrv.setFamily("CARF_ALTRV");
        malformedAltrv.setDirection("INBOUND");
        malformedAltrv.setSubject("Malformed ALTRV");
        malformedAltrv.setRawText("A. BROKEN\nD. FIXA 0000\nG. MISSING TIMING");
        productService.sendMessage(malformedAltrv);
        ProductDtos.FeedIngestRequest commandFeed = new ProductDtos.FeedIngestRequest();
        commandFeed.setSourceId("agentic-rq-tbl");
        commandFeed.setType("USNS");
        commandFeed.setRawPayload(envelope("(SVC TBL DOM)\n(SVC RQ DOM RQN KJFK HIST)"));
        productService.ingestFeed(commandFeed);
        DataIntegrityAgent agent = new DataIntegrityAgent();
        agent.productService = productService;

        AgentRunRequest request = new AgentRunRequest();
        request.setMissionId(missionId);
        request.setReservationId(reservationId);
        AgentRunResult result = agent.scan(request);

        assertTrue(result.getFindings().stream().anyMatch(finding -> "CONTRADICTORY_PIREP_WEATHER".equals(finding.getCategory())));
        assertTrue(result.getFindings().stream().anyMatch(finding -> "AMBIGUOUS_DOM2_SEMANTIC_REDUCTION".equals(finding.getCategory())));
        assertTrue(result.getFindings().stream().anyMatch(finding -> "MISSING_NOTAM_GEOMETRY".equals(finding.getCategory())));
        assertTrue(result.getFindings().stream().anyMatch(finding -> "AMBIGUOUS_ICAO_NOTAM_Q_FIELD".equals(finding.getCategory())));
        assertTrue(result.getFindings().stream().anyMatch(finding -> "MALFORMED_ALTRV_GRAMMAR".equals(finding.getCategory())));
        assertTrue(result.getFindings().stream().anyMatch(finding -> "MALFORMED_USNS_SERVICE_COMMAND".equals(finding.getCategory())));
        assertTrue(result.getFindings().stream().anyMatch(finding -> "USNS_TABLE_COMMAND_RETAINED_ONLY".equals(finding.getCategory())));
        assertTrue(result.getFindings().stream().anyMatch(finding -> "MISSING_GEOMETRY".equals(finding.getCategory())
                || "PIREP_RELEVANCE_MISMATCH".equals(finding.getCategory())));
        assertTrue(result.getFindings().stream().allMatch(finding -> finding.getCitations() != null && !finding.getCitations().isEmpty()));
    }

    @Test
    void scenarioGeneratorCreatesEngineUsableFixturesAndCalibrationLoadersReadCsv() {
        ScenarioFixtureGenerator generator = new ScenarioFixtureGenerator();
        ScenarioFixtureRequest fixtureRequest = new ScenarioFixtureRequest();
        fixtureRequest.setScenarioType("VIABLE_REROUTE");
        fixtureRequest.setMissionNumber("NXGEN-TEST");
        fixtureRequest.setIncludeMalformedInputs(true);
        ScenarioFixtureBundle bundle = generator.generate(fixtureRequest);

        assertEquals("VIABLE_REROUTE", bundle.getScenarioType());
        assertTrue(bundle.getCarfAltrv().contains("A. NXGEN-TEST"));
        assertFalse(bundle.getWeatherMessages().isEmpty());
        assertEquals("true", bundle.getExpectedSummary().get("malformedRetained"));

        AirspaceProductService productService = new AirspaceProductService(new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber(bundle.getMissionNumber());
        missionRequest.setTitle("Generated fixture mission");
        ProductDtos.MissionSummary mission = productService.createMission(missionRequest);
        ProductDtos.ReservationRequest reservationRequest = new ProductDtos.ReservationRequest();
        reservationRequest.setActor("planner");
        reservationRequest.setRawText(bundle.getCarfAltrv());
        String reservationId = productService.createReservation(mission.getId(), reservationRequest).getRecord().getId();
        for (String message : bundle.getWeatherMessages()) {
            ProductDtos.MessageRequest wx = new ProductDtos.MessageRequest();
            wx.setMissionId(mission.getId());
            wx.setReservationId(reservationId);
            wx.setFamily(message.startsWith("AIRMET") ? "AIRMET" : "SIGMET");
            wx.setDirection("INBOUND");
            wx.setSubject("Generated weather");
            wx.setRawText(message);
            productService.sendMessage(wx);
        }
        assertNotNull(productService.routeImpact(mission.getId(), reservationId));

        CalibrationFixtureLoader loader = new CalibrationFixtureLoader();
        WeatherOutcomeCalibrationDataset weather = loader.weatherOutcomesFromCsv("productType,severityScore,echoTopsFeet,leadTimeHours,blockedOutcome\nCWAP,0.8,45000,2,1");
        StormLifecycleCalibrationDataset lifecycle = loader.stormLifecycleFromCsv("cellId,phase,growthRate,movementKt,confidence\ncell-1,growing,0.4,25,0.8");
        SectorDemandCalibrationDataset demand = loader.sectorDemandFromCsv("sectorId,baselineCapacity,activeDemand,weatherCoverageRatio\nZNY,40,50,0.3");
        assertEquals(1, weather.records().size());
        assertEquals(1.0, weather.averageBlockedOutcome(), 0.01);
        assertEquals("growing", lifecycle.observations().get(0).getPhase());
        assertTrue(demand.averageDemandRatio() > 1.0);
        assertTrue(new LocalReplayValidationHarness().validateLocalReplay(bundle.getUsnsMessages()).isAccepted());
    }

    private AirspaceProductService seededProductService() {
        AirspaceProductService service = new AirspaceProductService(new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber("AGENT-1");
        missionRequest.setTitle("Agentic weather impact mission");
        ProductDtos.MissionSummary mission = service.createMission(missionRequest);

        ProductDtos.ReservationRequest reservationRequest = new ProductDtos.ReservationRequest();
        reservationRequest.setActor("planner");
        reservationRequest.setRawText("A. AGENT-1\nB. 1KC135/M\nC. JFK 200000Z\nD. 3000N15000W 3100N14900W\nE. JFK\nF. FL240-FL280\nG. WX AGENT TEST");
        String reservationId = service.createReservation(mission.getId(), reservationRequest).getRecord().getId();

        ProductDtos.MessageRequest sigmet = new ProductDtos.MessageRequest();
        sigmet.setMissionId(mission.getId());
        sigmet.setReservationId(reservationId);
        sigmet.setFamily("SIGMET");
        sigmet.setDirection("INBOUND");
        sigmet.setSubject("Agent SIGMET");
        sigmet.setRawText("SIGMET AGENT VALID 200000/200400 FROM 3000N15000W TO 3100N14900W EMBD TS MOV E 25KT TOP FL450 INTSF");
        service.sendMessage(sigmet);

        ProductDtos.MessageRequest notam = new ProductDtos.MessageRequest();
        notam.setMissionId(mission.getId());
        notam.setReservationId(reservationId);
        notam.setFamily("FDC");
        notam.setDirection("INBOUND");
        notam.setSubject("Agent FDC NOTAM");
        notam.setRawText("!FDC AGENT NOTAM AIRSPACE CLSD WI AN AREA 3000N15000W-3100N14900W SFC-FL260");
        service.sendMessage(notam);
        return service;
    }

    private AgenticOperationsService wiredService(AirspaceProductService productService) {
        AgenticOperationsService service = new AgenticOperationsService();
        service.weatherImpactWatchAgent = new WeatherImpactWatchAgent();
        service.weatherImpactWatchAgent.productService = productService;
        service.missionRiskAnalystAgent = new MissionRiskAnalystAgent();
        service.missionRiskAnalystAgent.productService = productService;
        OperationalDeltaService deltaService = new OperationalDeltaService();
        deltaService.productService = productService;
        service.missionRiskAnalystAgent.operationalDeltaService = deltaService;
        service.rerouteAnalystAgent = new RerouteAnalystAgent();
        service.rerouteAnalystAgent.productService = productService;
        service.coordinationDraftAgent = new CoordinationDraftAgent();
        service.coordinationDraftAgent.productService = productService;
        service.pilotBriefAgent = new PilotBriefAgent();
        service.pilotBriefAgent.productService = productService;
        service.dataIntegrityAgent = new DataIntegrityAgent();
        service.dataIntegrityAgent.productService = productService;
        service.replayAuditAgent = new ReplayAuditAgent();
        service.replayAuditAgent.productService = productService;
        service.replayAuditAgent.operationalDeltaService = deltaService;
        service.citationValidator = new AgentCitationValidator();
        service.policyEnforcer = new AgentPolicyEnforcer();
        service.evaluationService = new AgentEvaluationService();
        service.evaluationService.citationValidator = service.citationValidator;
        service.evaluationService.policyEnforcer = service.policyEnforcer;
        service.auditService = new AgentAuditService();
        return service;
    }

    private ProductDtos.DecisionEvaluateRequest decisionRequest(String missionId, String reservationId) {
        ProductDtos.DecisionEvaluateRequest request = new ProductDtos.DecisionEvaluateRequest();
        request.setMissionId(missionId);
        request.setReservationId(reservationId);
        request.setRoute(java.util.Arrays.asList(
                java.util.Arrays.asList(30.0, -150.5, 24000.0),
                java.util.Arrays.asList(30.5, -149.8, 25000.0),
                java.util.Arrays.asList(31.0, -149.0, 26000.0)));
        return request;
    }

    private String envelope(String body) {
        return "01GGNC07GP\n"
                + "CNS000 300334\n"
                + "GG KDZZNAXX\n"
                + "300334 KGPS\n"
                + MessageControlCharacters.STX + body
                + MessageControlCharacters.VT + MessageControlCharacters.ETX;
    }

    @Test
    void policyEnforcerRejectsAutonomousSendsAndWorkflowMutation() {
        AgentPolicyEnforcer enforcer = new AgentPolicyEnforcer();
        AgentRunResult result = AgentRunResult.builder()
                .id("agent-policy-test")
                .agentType("COORDINATION_DRAFT")
                .accepted(true)
                .recommendations(java.util.Arrays.asList(
                        AgentRecommendation.builder()
                                .id("send")
                                .action("SEND_EXTERNAL_MESSAGE")
                                .summary("send")
                                .humanApprovalRequired(false)
                                .build(),
                        AgentRecommendation.builder()
                                .id("approve")
                                .action("APPROVE_RESERVATION")
                                .summary("approve")
                                .humanApprovalRequired(false)
                                .build()))
                .build();

        java.util.List<String> diagnostics = enforcer.validate(result, AgentPolicy.builder().build());
        assertTrue(diagnostics.stream().anyMatch(value -> value.contains("external-send")));
        assertTrue(diagnostics.stream().anyMatch(value -> value.contains("workflow mutation")));
    }

    @Test
    void evaluationSummaryTracksCitationCoverageAndPolicyViolations() {
        AgentEvaluationService evaluationService = new AgentEvaluationService();
        evaluationService.citationValidator = new AgentCitationValidator();
        evaluationService.policyEnforcer = new AgentPolicyEnforcer();
        AgentRunResult result = AgentRunResult.builder()
                .id("agent-eval")
                .agentType("MISSION_RISK")
                .accepted(true)
                .findings(java.util.Arrays.asList(
                        AgentFinding.builder()
                                .id("cited")
                                .category("MISSION_RISK")
                                .severity("INFO")
                                .message("cited")
                                .citations(java.util.Collections.singletonList(AgentSourceCitation.builder()
                                        .sourceFamily("WEATHER")
                                        .sourceId("wx-1")
                                        .build()))
                                .build(),
                        AgentFinding.builder()
                                .id("uncited")
                                .category("MISSION_RISK")
                                .severity("INFO")
                                .message("uncited")
                                .build()))
                .recommendations(java.util.Collections.singletonList(AgentRecommendation.builder()
                        .id("send")
                        .action("SEND_EXTERNAL_MESSAGE")
                        .summary("send")
                        .humanApprovalRequired(false)
                        .build()))
                .build();

        AgentEvaluationSummary summary = evaluationService.evaluate(result, AgentPolicy.builder().build());
        assertFalse(summary.isAccepted());
        assertTrue(summary.getCitationCoverage() < 1.0);
        assertTrue(summary.getPolicyViolationCount() >= 1);
        assertEquals(Integer.valueOf(1), summary.getSourceFamilyCounts().get("WEATHER"));
    }
}
