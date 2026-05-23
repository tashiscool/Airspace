package org.tash.extensions.product;

import io.quarkus.test.junit.QuarkusTest;
import jakarta.inject.Inject;
import org.junit.jupiter.api.Test;
import org.tash.extensions.engine.OperationalDecisionEngine;
import org.tash.extensions.engine.OperationalDecisionRequest;
import org.tash.extensions.feed.OperationalFeedEnvelope;
import org.tash.extensions.feed.OperationalFeedIngestResult;
import org.tash.extensions.feed.OperationalFeedType;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.application.ProductPersistenceService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.InMemoryReservationWorkflowRepository;
import org.tash.extensions.workflow.ReservationDraft;
import org.tash.extensions.workflow.ReservationWorkflowRecord;
import org.tash.extensions.workflow.ReservationWorkflowService;
import org.tash.extensions.workflow.ReservationWorkflowState;

import java.time.ZonedDateTime;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

@QuarkusTest
class ProductPersistenceRuntimeTest {
    @Inject
    ProductPersistenceService persistenceService;

    @Test
    void productServiceSearchRehydratesPersistedFeedTransactions() {
        AirspaceProductService service = new AirspaceProductService(
                new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()),
                persistenceService,
                true);
        ProductDtos.FeedIngestRequest feed = new ProductDtos.FeedIngestRequest();
        feed.setSourceId("runtime-grammar-" + System.nanoTime());
        feed.setType("USNS");
        feed.setRawPayload(envelope("!DCA LDN RWY 10/28 CLSD 1012211200-1012211300\n"
                + "(A1001/26 NOTAMN Q) /QRTCA/IV/BO/W/000/180/3000N15000W005 A) KZNY B) 2601011200 C) PERM E) TEST)"));

        String artifactId = service.ingestFeed(feed).getResults().get(0).getEnvelope().getId();

        assertTrue(service.search("DOM2.SURFACE.CLOSED").stream().anyMatch(result ->
                "feed-transaction".equals(result.getType())
                        && result.getRoute().equals("/feed/" + artifactId)
                        && result.getSnippet().contains("RWY")));
        assertTrue(service.search("QRTCA").stream().anyMatch(result ->
                "feed-transaction".equals(result.getType())
                        && result.getRoute().equals("/feed/" + artifactId)
                        && result.getSnippet().contains("GEOMETRY")));
    }

    @Test
    void productPersistenceServiceStoresOperationalAggregatesThroughFlywaySchema() {
        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber("PERSIST-" + System.nanoTime());
        missionRequest.setTitle("Persisted Mission");
        missionRequest.setRawText("A. TEST");

        ProductDtos.MissionSummary mission = persistenceService.createMission(
                missionRequest, java.util.UUID.randomUUID().toString());

        ProductDtos.MessageSummary message = ProductDtos.MessageSummary.builder()
                .id(java.util.UUID.randomUUID().toString())
                .missionId(mission.getId())
                .family("USNS")
                .direction("INBOUND")
                .status("ACCEPTED")
                .subject("Persistence smoke")
                .rawText("SVC RQ")
                .createdAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                .build();
        ProductDtos.MessageSummary savedMessage = persistenceService.saveMessage(message);

        String reservationId = java.util.UUID.randomUUID().toString();
        ProductDtos.ReservationSummary savedReservation = persistenceService.saveReservation(mission.getId(),
                ReservationWorkflowRecord.builder()
                        .id(reservationId)
                        .state(ReservationWorkflowState.DRAFT)
                        .draft(ReservationDraft.builder()
                                .id(reservationId)
                                .rawText("A. TEST\nB. 1F22/I")
                                .createdBy("planner")
                                .updatedBy("planner")
                                .createdAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                                .updatedAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                                .build())
                        .build());

        java.util.UUID decisionId = persistenceService.saveDecision(new OperationalDecisionEngine()
                .evaluate(OperationalDecisionRequest.builder()
                        .decisionTime(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                        .build()));
        ProductDtos.FeedArtifactSummary feed = persistenceService.saveFeedArtifact(OperationalFeedIngestResult.builder()
                .envelope(OperationalFeedEnvelope.builder()
                        .id(java.util.UUID.randomUUID().toString())
                        .sourceId("runtime-test")
                        .type(OperationalFeedType.WEATHER)
                        .rawPayload("METAR KJFK 200000Z 18012KT 2SM RA BKN010")
                        .receivedAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                        .build())
                .accepted(true)
                .rawPayloadHash("hash")
                .downstreamArtifactIds(Collections.singletonList("weather:KJFK"))
                .warnings(Collections.singletonList("stored"))
                .build());
        ProductDtos.HistoryEventSummary history = persistenceService.saveHistory(ProductDtos.HistoryEventSummary.builder()
                .id(java.util.UUID.randomUUID().toString())
                .aggregateType("feed")
                .aggregateId(feed.getId())
                .eventType("FEED_INGESTED")
                .actor("system")
                .note("accepted")
                .createdAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                .build());
        ProductDtos.ReferencePointRequest referencePointRequest = new ProductDtos.ReferencePointRequest();
        referencePointRequest.setIdentifier("RUNTIMEFIX-" + System.nanoTime());
        referencePointRequest.setPointType("FIX");
        referencePointRequest.setLatitude(37.5);
        referencePointRequest.setLongitude(-76.5);
        referencePointRequest.setAltitudeFeet(0.0);
        referencePointRequest.setSource("runtime-test");
        ProductDtos.ReferencePointSummary referencePoint = persistenceService.saveReferencePoint(referencePointRequest);
        ProductDtos.ReservationSupplementRequest supplementRequest = new ProductDtos.ReservationSupplementRequest();
        supplementRequest.setKind("APREQ");
        supplementRequest.setStatus("REQUESTED");
        supplementRequest.setTitle("APREQ");
        supplementRequest.setText("runtime apreq");
        supplementRequest.setActor("planner");
        ProductDtos.ReservationSupplementSummary supplement = persistenceService.saveSupplement(reservationId, supplementRequest);
        ProductDtos.ReservationSupplementRequest notamSupplement = new ProductDtos.ReservationSupplementRequest();
        notamSupplement.setKind("NOTAM");
        notamSupplement.setStatus("DRAFT");
        notamSupplement.setTitle("Domestic NOTAM");
        notamSupplement.setText("runtime notam");
        ProductDtos.ReservationSupplementSummary notam = persistenceService.saveSupplement(reservationId, notamSupplement);
        ProductDtos.ReservationSupplementRequest approvalSupplement = new ProductDtos.ReservationSupplementRequest();
        approvalSupplement.setKind("APPROVAL");
        approvalSupplement.setStatus("OPEN");
        approvalSupplement.setTitle("Supervisor Approval");
        approvalSupplement.setText("runtime approval");
        approvalSupplement.setActor("supervisor");
        ProductDtos.ReservationSupplementSummary approval = persistenceService.saveSupplement(reservationId, approvalSupplement);

        assertTrue(persistenceService.missions().stream()
                .anyMatch(saved -> mission.getId().equals(saved.getId())));
        assertTrue(persistenceService.mission(mission.getId()).getReservations().stream()
                .anyMatch(saved -> savedReservation.getId().equals(saved.getId())));
        assertEquals("ACCEPTED", savedMessage.getStatus());
        assertTrue(persistenceService.feedArtifacts().stream().anyMatch(saved -> feed.getId().equals(saved.getId())));
        ProductDtos.FeedArtifactSummary reloadedFeed = persistenceService.feedArtifact(feed.getId());
        assertEquals("METAR KJFK 200000Z 18012KT 2SM RA BKN010", reloadedFeed.getRawPayload());
        assertEquals(Collections.singletonList("weather:KJFK"), reloadedFeed.getDownstreamArtifactIds());
        assertEquals("SVC RQ", persistenceService.message(savedMessage.getId()).getRawText());
        ProductDtos.DecisionSummary persistedDecision = persistenceService.decision(decisionId.toString());
        assertNotNull(persistedDecision.getAction());
        assertEquals(decisionId.toString(), persistedDecision.getId());
        assertNotNull(persistedDecision.getResultJson());
        assertNotNull(persistedDecision.getAuditJson());
        assertNotNull(persistedDecision.getReplayJson());
        assertEquals(persistedDecision.getAction(), persistenceService.decisionResult(decisionId.toString()).getAction().name());
        assertEquals(persistedDecision.getAction(), persistenceService.decisionReplayBundle(decisionId.toString()).getExpectedAction());
        assertEquals(persistenceService.decisionReplayBundle(decisionId.toString()).getExpectedResultHash(),
                persistenceService.decisionAuditEnvelope(decisionId.toString()).getResultHash());
        assertTrue(persistenceService.history().stream().anyMatch(saved -> history.getId().equals(saved.getId())));
        ProductDtos.HistoryEventSummary missionHistory = persistenceService.saveHistory(ProductDtos.HistoryEventSummary.builder()
                .id(java.util.UUID.randomUUID().toString())
                .aggregateType("mission")
                .aggregateId(mission.getId())
                .eventType("MISSION_REVIEWED")
                .actor("planner")
                .note("included in mission detail")
                .createdAt(ZonedDateTime.parse("2026-05-20T00:00:01Z"))
                .build());
        assertTrue(persistenceService.mission(mission.getId()).getHistory().stream()
                .anyMatch(saved -> missionHistory.getId().equals(saved.getId())));
        assertTrue(persistenceService.search("Persisted Mission").stream().anyMatch(result -> "mission".equals(result.getType())));
        assertTrue(persistenceService.search("Persistence smoke").stream().anyMatch(result -> "message".equals(result.getType())));
        assertTrue(persistenceService.search("runtime-test").stream().anyMatch(result -> "feed".equals(result.getType())));
        assertTrue(persistenceService.search("MISSION_REVIEWED").stream().anyMatch(result -> "history".equals(result.getType())));
        assertTrue(persistenceService.referencePoints("FIX").stream()
                .anyMatch(saved -> referencePoint.getIdentifier().equals(saved.getIdentifier())));
        assertTrue(persistenceService.supplements(reservationId).stream()
                .anyMatch(saved -> supplement.getId().equals(saved.getId()) && "APREQ".equals(saved.getKind())));
        assertTrue(persistenceService.supplements(reservationId).stream()
                .anyMatch(saved -> notam.getId().equals(saved.getId()) && "NOTAM".equals(saved.getKind())));
        assertTrue(persistenceService.supplements(reservationId).stream()
                .anyMatch(saved -> approval.getId().equals(saved.getId()) && "APPROVAL".equals(saved.getKind())));
        java.util.Map<String, Double> metrics = persistenceService.metrics();
        assertTrue(metrics.get("product.missions") >= 1.0);
        assertTrue(metrics.get("product.reservations") >= 1.0);
        assertTrue(metrics.get("product.messages") >= 1.0);
        assertTrue(metrics.get("product.feedArtifacts") >= 1.0);
        assertTrue(metrics.get("product.decisions") >= 1.0);
        assertTrue(metrics.get("product.supplements") >= 3.0);
        assertTrue(metrics.get("product.notams") >= 1.0);
        assertTrue(metrics.get("product.apreqs") >= 1.0);
        assertTrue(metrics.get("product.approvals") >= 1.0);
        assertNotNull(decisionId);
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
