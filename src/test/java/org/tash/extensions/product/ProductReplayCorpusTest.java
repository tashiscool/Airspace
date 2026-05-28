package org.tash.extensions.product;

import org.junit.jupiter.api.Test;
import org.tash.extensions.feed.OperationalFeedBatchResult;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.InMemoryReservationWorkflowRepository;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class ProductReplayCorpusTest {
    @Test
    void productReplayCorpusFlowsThroughFeedAndDecisionWithoutExternalDependencies() throws Exception {
        AirspaceProductService service = new AirspaceProductService(
                new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        String usns = resource("/scenarios/product-replay/usns-mixed-weather.txt");
        String carf = resource("/scenarios/product-replay/carf-altrv.txt");

        ProductDtos.FeedIngestRequest feed = new ProductDtos.FeedIngestRequest();
        feed.setSourceId("product-replay");
        feed.setType("USNS");
        feed.setRawPayload(usns);
        OperationalFeedBatchResult batch = service.ingestFeed(feed);

        assertFalse(batch.getResults().isEmpty());
        assertFalse(service.feedArtifacts().isEmpty());
        assertFalse(service.feedTransactions(batch.getResults().get(0).getEnvelope().getId()).isEmpty());

        ProductDtos.DecisionEvaluateRequest decision = new ProductDtos.DecisionEvaluateRequest();
        decision.setDecisionTime("2026-05-20T00:00:00Z");
        decision.setRawUsnsMessages(Collections.singletonList(usns));
        decision.setRawCarfMessages(Collections.singletonList(carf));
        ProductDtos.DecisionSummary summary = service.evaluateDecision(decision);

        assertNotNull(summary.getId());
        assertNotNull(summary.getAction());
        assertNotNull(service.decisionResult(summary.getId()).getReplayBundle());
        assertTrue(service.metrics().get("product.feedArtifacts") >= 1.0);
        assertTrue(service.metrics().get("product.decisions") >= 1.0);
    }

    @Test
    void feedTransactionsExposeTypedIcaoAndCanadianNotamFields() {
        AirspaceProductService service = new AirspaceProductService(
                new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        ProductDtos.FeedIngestRequest feed = new ProductDtos.FeedIngestRequest();
        feed.setSourceId("grammar-parity");
        feed.setType("USNS");
        feed.setRawPayload(envelope("!DCA LDN RWY 10/28 CLSD 1012211200-1012211300\n"
                + "(A0001/26 NOTAMN Q) /QRTCA/IV/BO/W/000/180/3000N15000W005 A) KZNY B) 2601011200 C) PERM E) TEST)\n"
                + "(A0002/26 NOTAMC A) KZNY E) CANCEL TEST)\n"
                + "(1234/26 NOTAMJ A) CYUL E) CANADIAN TEST TIL APRX 2601011200)\n"
                + "(SVC RQ DOM RQN KJFK HIST)\n"
                + "(SVC TBL ROUTING UPDATE)\n"
                + "GENOT RWA TEST ADMIN MESSAGE"));

        OperationalFeedBatchResult batch = service.ingestFeed(feed);
        String artifactId = batch.getResults().get(0).getEnvelope().getId();
        java.util.List<ProductDtos.FeedTransactionSummary> transactions = service.feedTransactions(artifactId);

        assertTrue(transactions.stream().anyMatch(transaction ->
                "NOTAMN".equals(transaction.getNotamType())
                        && "QRTCA".equals(transaction.getNotamQCode())
                        && transaction.isNotamHasGeometry()
                        && transaction.isNotamPermanentEnd()));
        assertTrue(transactions.stream().anyMatch(transaction ->
                "RWY".equals(transaction.getDomesticNotamKeyword())
                        && "DOM2.SURFACE.CLOSED".equals(transaction.getDomesticNotamReducerRuleId())
                        && "LC".equals(transaction.getDomesticNotamQ23())));
        assertTrue(transactions.stream().anyMatch(transaction ->
                "NOTAMC".equals(transaction.getNotamType())
                        && !transaction.isNotamHasGeometry()
                        && transaction.getWarnings().stream().anyMatch(warning -> warning.contains("No compact coordinate"))));
        assertTrue(transactions.stream().anyMatch(transaction ->
                "NOTAMJ".equals(transaction.getNotamType())
                        && "CYUL".equals(transaction.getNotamAffectedLocation())));
        assertTrue(transactions.stream().anyMatch(transaction ->
                "REQUEST".equals(transaction.getServiceCommandType())
                        && "DOM".equals(transaction.getServiceCommandDomain())
                        && "RQN".equals(transaction.getServiceCommandOperation())
                        && "KJFK".equals(transaction.getServiceCommandLocation())
                        && transaction.isServiceCommandHistory()));
        assertTrue(transactions.stream().anyMatch(transaction ->
                "TABLE".equals(transaction.getServiceCommandType())
                        && "ROUTING".equals(transaction.getServiceCommandDomain())
                        && "UPDATE".equals(transaction.getServiceCommandOperation())));
        assertTrue(transactions.stream().anyMatch(transaction ->
                "GENOT".equals(transaction.getType())
                        && "genot-admin-message".equals(transaction.getFamilySemantic())
                        && "RWA".equals(transaction.getFamilyGenotSeries())));
        assertTrue(service.search("DOM2.SURFACE.CLOSED").stream().anyMatch(result ->
                "feed-transaction".equals(result.getType())
                        && result.getRoute().equals("/feed/" + artifactId)
                        && result.getSnippet().contains("RWY")));
        assertTrue(service.search("QRTCA").stream().anyMatch(result ->
                "feed-transaction".equals(result.getType())
                        && result.getRoute().equals("/feed/" + artifactId)
                        && result.getSnippet().contains("GEOMETRY")));
        assertTrue(service.search("RQN").stream().anyMatch(result ->
                "feed-transaction".equals(result.getType())
                        && result.getRoute().equals("/feed/" + artifactId)
                        && result.getSnippet().contains("REQUEST")));
        assertTrue(service.search("genot-admin-message").stream().anyMatch(result ->
                "feed-transaction".equals(result.getType())
                        && result.getRoute().equals("/feed/" + artifactId)
                        && result.getSnippet().contains("GENOT")));
    }

    @Test
    void productReplayCorpusExercisesOperatorGuidanceSurfaces() throws Exception {
        AirspaceProductService service = new AirspaceProductService(
                new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        Map<String, String> expected = expected("/scenarios/product-replay/expected-summary.txt");

        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber("PRODUCT-REPLAY");
        missionRequest.setTitle("Product replay weather safety scenario");
        ProductDtos.MissionSummary mission = service.createMission(missionRequest);

        ProductDtos.ReservationRequest reservationRequest = new ProductDtos.ReservationRequest();
        reservationRequest.setActor("planner");
        reservationRequest.setRawText(resource("/scenarios/product-replay/carf-altrv.txt"));
        String reservationId = service.createReservation(mission.getId(), reservationRequest).getRecord().getId();

        sendWeatherMessage(service, mission.getId(), reservationId, "PIREP", "Product replay PIREP",
                "UA /OV 3000N15000W/TM 2000/FL240/TP B738/TB MOD/RM PRODUCT DEMO PIREP");
        sendWeatherMessage(service, mission.getId(), reservationId, "SIGMET", "Product replay SIGMET",
                "SIGMET DEMO 1 VALID 200000/200400 FROM 3000N15000W TO 3100N14900W EMBD TS MOV E 25KT TOP FL450 INTSF");
        sendWeatherMessage(service, mission.getId(), reservationId, "METAR", "Product replay METAR",
                "METAR KJFK 200000Z 18012G22KT 1/2SM +TSRA BKN004 OVC010 18/16 A2992");
        sendWeatherMessage(service, mission.getId(), reservationId, "FDC", "Product replay FDC NOTAM",
                "!FDC PRODUCT NOTAM AIRSPACE CLSD WI AN AREA 3000N15000W-3100N14900W SFC-FL260");

        ProductDtos.MissionWeatherVerdictSummary verdict = service.missionWeatherVerdict(mission.getId());
        ProductDtos.RouteImpactSummary impact = service.routeImpact(mission.getId(), reservationId);
        ProductDtos.DecisionEvaluateRequest contextualDecision = new ProductDtos.DecisionEvaluateRequest();
        contextualDecision.setDecisionTime("2026-05-20T00:00:00Z");
        contextualDecision.setRawCarfMessages(Collections.singletonList(resource("/scenarios/product-replay/carf-altrv.txt")));
        contextualDecision.setMissionId(mission.getId());
        contextualDecision.setReservationId(reservationId);
        ProductDtos.DecisionSummary contextualSummary = service.evaluateDecision(contextualDecision);
        OperationalDecisionResult reloadedResult = service.decisionResult(contextualSummary.getId());
        ProductDtos.PirepRelevanceRequest relevanceRequest = new ProductDtos.PirepRelevanceRequest();
        relevanceRequest.setReservationId(reservationId);
        relevanceRequest.setLowerAltitudeFeet(22000.0);
        relevanceRequest.setUpperAltitudeFeet(28000.0);
        relevanceRequest.setRecencyMinutes(1440);
        ProductDtos.PirepRelevanceResult pireps = service.relevantPireps(mission.getId(), relevanceRequest);
        ProductDtos.CoordinationDraftRequest draftRequest = new ProductDtos.CoordinationDraftRequest();
        draftRequest.setReservationId(reservationId);
        draftRequest.setActor("planner");
        ProductDtos.CoordinationDraftSummary draft = service.coordinationDraft(mission.getId(), draftRequest);
        ProductDtos.PilotBriefSummary brief = service.pilotBrief(mission.getId(), null);
        ProductDtos.AffectedMissionSummary affected = service.affectedMissions(null, 25).stream()
                .filter(item -> mission.getId().equals(item.getMissionId()))
                .findFirst()
                .orElseThrow();
        assertTrue(Arrays.asList(expected.getOrDefault("operatorSurfaces", "").split(",")).containsAll(Arrays.asList(
                "mission-verdict",
                "route-impact",
                "pirep-relevance",
                "coordination-draft",
                "pilot-brief",
                "affected-mission-metrics")));

        assertTrue(verdict.getSourceCount() >= Integer.parseInt(expected.getOrDefault("minSources", "4")));
        assertTrue(Arrays.asList(expected.getOrDefault("allowedActions", "AVOID,REROUTE,BLOCKED,CAUTION,MONITOR").split(","))
                .contains(verdict.getAction()), "Unexpected verdict action " + verdict.getAction());
        assertEquals(mission.getId(), impact.getMissionId());
        assertFalse(impact.getSourceRefs().isEmpty(), "Route impact should retain source refs for audit");
        assertTrue(impact.getSourceRefs().stream().anyMatch(ref -> ref.startsWith("FDC:")));
        assertTrue(impact.getImpactedSegments().stream().anyMatch(segment -> segment.contains("NOTAM constraint")));
        assertTrue(impact.getOriginalRouteDistanceNm() > 0.0);
        assertFalse(impact.getWhyRerouteTrace().isEmpty());
        assertFalse(impact.getAvoidanceCandidates().isEmpty());
        assertFalse(impact.getCandidateComparisons().isEmpty());
        assertNotNull(contextualSummary.getRouteImpact());
        assertEquals(mission.getId(), contextualSummary.getRouteImpact().getMissionId());
        assertFalse(contextualSummary.getRouteImpact().getCandidateComparisons().isEmpty());
        assertNotNull(reloadedResult);
        assertNotNull(reloadedResult.getReplayBundle());
        assertTrue(pireps.getTotalPireps() >= 1);
        assertTrue(pireps.getAverageRelevanceScore() > 0.0);
        assertTrue(verdict.getSources().stream().anyMatch(source -> "FDC".equals(source.getFamily())
                && source.getRationale().contains("NOTAM constraint")));
        assertTrue(draft.getRawText().contains("WEATHER COORDINATION"));
        assertTrue(draft.getSourceRefs().stream().anyMatch(ref -> ref.contains("SIGMET") || ref.contains("PIREP") || ref.contains("METAR") || ref.contains("FDC")));
        assertTrue(brief.getPrintableText().contains("AIRSPACE PILOT BRIEF"));
        assertTrue(brief.getPrintableText().contains("SOURCE DRIVERS"));
        assertTrue(brief.getSourceSummaryLines().stream().anyMatch(line -> line.contains("NOTAM")));
        assertTrue(brief.getPrintableText().contains("TRACE:"));
        assertTrue(affected.getGuidanceLatencySeconds() >= 0);
        assertFalse(affected.getSourceRefs().isEmpty());
        assertFalse(affected.getRouteCoordinates().isEmpty());
        assertTrue(affected.getRerouteCandidateCount() >= 1);
        assertTrue(affected.getRerouteAdditionalCostUsd() > 0.0);
        assertTrue(service.metrics().get("product.weather.affectedMissions") >= 1.0);
    }

    private void sendWeatherMessage(AirspaceProductService service,
                                    String missionId,
                                    String reservationId,
                                    String family,
                                    String subject,
                                    String rawText) {
        ProductDtos.MessageRequest request = new ProductDtos.MessageRequest();
        request.setMissionId(missionId);
        request.setReservationId(reservationId);
        request.setFamily(family);
        request.setDirection("INBOUND");
        request.setSubject(subject);
        request.setRawText(rawText);
        request.setActor("product-replay");
        service.sendMessage(request);
    }

    private Map<String, String> expected(String path) throws Exception {
        Map<String, String> values = new LinkedHashMap<>();
        for (String line : resource(path).split("\\R")) {
            if (line.trim().isEmpty() || line.startsWith("#") || !line.contains("=")) {
                continue;
            }
            String[] pair = line.split("=", 2);
            values.put(pair[0].trim(), pair[1].trim());
        }
        return values;
    }

    private String resource(String path) throws Exception {
        return new String(getClass().getResourceAsStream(path).readAllBytes(), StandardCharsets.UTF_8);
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
