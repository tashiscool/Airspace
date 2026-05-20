package org.tash.extensions.product.application;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.eclipse.microprofile.config.inject.ConfigProperty;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.OperationalDecisionEngine;
import org.tash.extensions.engine.OperationalDecisionAuditEnvelope;
import org.tash.extensions.engine.OperationalDecisionRequest;
import org.tash.extensions.engine.OperationalDecisionReplayBundle;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.feed.InMemoryOperationalFeedSource;
import org.tash.extensions.feed.LocalReferenceDataSyncAdapter;
import org.tash.extensions.feed.OperationalFeedBatchResult;
import org.tash.extensions.feed.OperationalFeedEnvelope;
import org.tash.extensions.feed.OperationalFeedIngestResult;
import org.tash.extensions.feed.OperationalFeedIngestService;
import org.tash.extensions.feed.OperationalFeedType;
import org.tash.extensions.feed.ReferenceDataImportRecord;
import org.tash.extensions.feed.ReferenceDataSyncResult;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.ReservationWorkflowRecord;
import org.tash.extensions.workflow.ReservationWorkflowResult;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.UUID;

@ApplicationScoped
public class AirspaceProductService {
    private final ProductRecordStore store = new ProductRecordStore();
    private final ReservationWorkflowService workflowService;
    private final ProductPersistenceService persistenceService;
    private final boolean persistenceEnabled;
    private final OperationalFeedIngestService feedIngestService = new OperationalFeedIngestService();
    private final OperationalDecisionEngine decisionEngine = new OperationalDecisionEngine();

    @Inject
    public AirspaceProductService(ReservationWorkflowService workflowService,
                                  ProductPersistenceService persistenceService,
                                  @ConfigProperty(name = "airspace.product.persistence.enabled", defaultValue = "false")
                                  boolean persistenceEnabled) {
        this.workflowService = workflowService;
        this.persistenceService = persistenceService;
        this.persistenceEnabled = persistenceEnabled;
    }

    public AirspaceProductService(ReservationWorkflowService workflowService) {
        this.workflowService = workflowService;
        this.persistenceService = null;
        this.persistenceEnabled = false;
    }

    public List<ProductDtos.MissionSummary> missions() {
        if (usePersistence()) {
            return persistenceService.missions();
        }
        return new ArrayList<>(store.missions.values());
    }

    public ProductDtos.MissionDetail mission(String id) {
        if (usePersistence()) {
            return persistenceService.mission(id);
        }
        ProductDtos.MissionSummary mission = store.missions.get(id);
        if (mission == null) {
            throw new IllegalArgumentException("Unknown mission: " + id);
        }
        List<ProductDtos.ReservationSummary> reservations = new ArrayList<>();
        synchronized (store.reservations) {
            for (ReservationWorkflowRecord record : store.reservations.values()) {
                if (id.equals(store.reservationMissionIds.get(record.getId()))) {
                    reservations.add(reservationSummary(id, record));
                }
            }
        }
        List<ProductDtos.MessageSummary> messages = new ArrayList<>();
        synchronized (store.messages) {
            for (ProductDtos.MessageSummary message : store.messages.values()) {
                if (id.equals(message.getMissionId())) {
                    messages.add(message);
                }
            }
        }
        return ProductDtos.MissionDetail.builder()
                .mission(mission)
                .reservations(reservations)
                .messages(messages)
                .history(store.historyFor("mission", id))
                .build();
    }

    public ProductDtos.MissionSummary createMission(ProductDtos.MissionRequest request) {
        ProductDtos.MissionRequest safe = request == null ? new ProductDtos.MissionRequest() : request;
        String id = UUID.randomUUID().toString();
        ZonedDateTime now = ZonedDateTime.now(ZoneOffset.UTC);
        ProductDtos.MissionSummary mission = ProductDtos.MissionSummary.builder()
                .id(id)
                .missionNumber(blank(safe.getMissionNumber()) ? "MISSION-" + id.substring(0, 8) : safe.getMissionNumber())
                .title(safe.getTitle())
                .status("DRAFT")
                .reservationCount(0)
                .updatedAt(now)
                .build();
        store.missions.put(id, mission);
        if (usePersistence()) {
            safe.setMissionNumber(mission.getMissionNumber());
            safe.setTitle(mission.getTitle());
            persistenceService.createMission(safe, id);
        }
        history("mission", id, "MISSION_CREATED", actor(safe.getActor()), safe.getRawText());
        return mission;
    }

    public ProductDtos.MissionSummary lockMission(String id, String actor) {
        ProductDtos.MissionSummary mission = requireMission(id);
        ProductDtos.MissionSummary locked = mission.toBuilder()
                .lockedBy(actor(actor))
                .lockedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .build();
        store.missions.put(id, locked);
        if (usePersistence()) {
            persistenceService.saveMission(locked);
        }
        history("mission", id, "MISSION_LOCKED", actor(actor), null);
        return locked;
    }

    public ProductDtos.MissionSummary unlockMission(String id, String actor) {
        ProductDtos.MissionSummary mission = requireMission(id);
        ProductDtos.MissionSummary unlocked = mission.toBuilder().lockedBy(null).lockedAt(null).build();
        store.missions.put(id, unlocked);
        if (usePersistence()) {
            persistenceService.saveMission(unlocked);
        }
        history("mission", id, "MISSION_UNLOCKED", actor(actor), null);
        return unlocked;
    }

    public ReservationWorkflowResult createReservation(String missionId, ProductDtos.ReservationRequest request) {
        ProductDtos.ReservationRequest safe = request == null ? new ProductDtos.ReservationRequest() : request;
        requireMission(missionId);
        ReservationWorkflowResult result = workflowService.createDraft(safe.getRawText(), actor(safe.getActor()));
        store.reservations.put(result.getRecord().getId(), result.getRecord());
        store.reservationMissionIds.put(result.getRecord().getId(), missionId);
        ProductDtos.MissionSummary mission = requireMission(missionId);
        store.missions.put(missionId, mission.toBuilder().reservationCount(mission.getReservationCount() + 1).build());
        if (usePersistence()) {
            persistenceService.saveReservation(missionId, result.getRecord());
        }
        history("mission", missionId, "RESERVATION_CREATED", actor(safe.getActor()), result.getRecord().getId());
        return result;
    }

    public ReservationWorkflowResult updateReservation(String id, ProductDtos.ReservationRequest request) {
        ProductDtos.ReservationRequest safe = request == null ? new ProductDtos.ReservationRequest() : request;
        return recordReservationWorkflowResult(id,
                workflowService.updateDraft(id, safe.getRawText(), actor(safe.getActor())),
                "RESERVATION_UPDATED", actor(safe.getActor()), safe.getNote());
    }

    public ReservationWorkflowResult validateReservation(String id, ProductDtos.ReservationRequest request) {
        return recordReservationWorkflowResult(id,
                workflowService.validate(id, actor(request == null ? null : request.getActor())),
                "RESERVATION_VALIDATED", actor(request == null ? null : request.getActor()), null);
    }

    public ReservationWorkflowResult submitReservation(String id, ProductDtos.ReservationRequest request) {
        return recordReservationWorkflowResult(id,
                workflowService.submit(id, actor(request == null ? null : request.getActor())),
                "RESERVATION_SUBMITTED", actor(request == null ? null : request.getActor()), null);
    }

    public ReservationWorkflowResult approveReservation(String id, ProductDtos.ReservationRequest request) {
        return recordReservationWorkflowResult(id,
                workflowService.approve(id, actor(request == null ? null : request.getActor())),
                "RESERVATION_APPROVED", actor(request == null ? null : request.getActor()), null);
    }

    public ReservationWorkflowResult rejectReservation(String id, ProductDtos.ReservationRequest request) {
        ProductDtos.ReservationRequest safe = request == null ? new ProductDtos.ReservationRequest() : request;
        return recordReservationWorkflowResult(id,
                workflowService.reject(id, actor(safe.getActor()), safe.getNote()),
                "RESERVATION_REJECTED", actor(safe.getActor()), safe.getNote());
    }

    public ReservationWorkflowResult cancelReservation(String id, ProductDtos.ReservationRequest request) {
        ProductDtos.ReservationRequest safe = request == null ? new ProductDtos.ReservationRequest() : request;
        return recordReservationWorkflowResult(id,
                workflowService.cancel(id, actor(safe.getActor()), safe.getNote()),
                "RESERVATION_CANCELLED", actor(safe.getActor()), safe.getNote());
    }

    public ReservationWorkflowResult completeReservation(String id, ProductDtos.ReservationRequest request) {
        return recordReservationWorkflowResult(id,
                workflowService.complete(id, actor(request == null ? null : request.getActor())),
                "RESERVATION_COMPLETED", actor(request == null ? null : request.getActor()), null);
    }

    public ReservationWorkflowResult lockReservation(String id, ProductDtos.ReservationRequest request) {
        return recordReservationWorkflowResult(id,
                workflowService.lock(id, actor(request == null ? null : request.getActor())),
                "RESERVATION_LOCKED", actor(request == null ? null : request.getActor()), null);
    }

    public ReservationWorkflowResult unlockReservation(String id, ProductDtos.ReservationRequest request) {
        return recordReservationWorkflowResult(id,
                workflowService.unlock(id, actor(request == null ? null : request.getActor())),
                "RESERVATION_UNLOCKED", actor(request == null ? null : request.getActor()), null);
    }

    public ProductDtos.MessageSummary sendMessage(ProductDtos.MessageRequest request) {
        ProductDtos.MessageRequest safe = request == null ? new ProductDtos.MessageRequest() : request;
        ProductDtos.MessageSummary message = ProductDtos.MessageSummary.builder()
                .id(UUID.randomUUID().toString())
                .missionId(safe.getMissionId())
                .reservationId(safe.getReservationId())
                .family(value(safe.getFamily(), "USNS"))
                .direction(value(safe.getDirection(), "OUTBOUND"))
                .status("QUEUED")
                .subject(safe.getSubject())
                .rawText(safe.getRawText())
                .createdAt(ZonedDateTime.now(ZoneOffset.UTC))
                .build();
        store.messages.put(message.getId(), message);
        if (usePersistence()) {
            persistenceService.saveMessage(message);
        }
        history("message", message.getId(), "MESSAGE_QUEUED", actor(safe.getActor()), message.getSubject());
        return message;
    }

    public ProductDtos.MessageSummary replyMessage(String id, ProductDtos.MessageRequest request) {
        ProductDtos.MessageSummary original = message(id);
        ProductDtos.MessageRequest safe = request == null ? new ProductDtos.MessageRequest() : request;
        ProductDtos.MessageRequest reply = new ProductDtos.MessageRequest();
        reply.setMissionId(value(safe.getMissionId(), original.getMissionId()));
        reply.setReservationId(value(safe.getReservationId(), original.getReservationId()));
        reply.setFamily(value(safe.getFamily(), original.getFamily()));
        reply.setDirection(value(safe.getDirection(), "OUTBOUND"));
        reply.setSubject(value(safe.getSubject(), "RE: " + value(original.getSubject(), original.getFamily())));
        reply.setRawText(safe.getRawText());
        reply.setActor(safe.getActor());
        ProductDtos.MessageSummary created = sendMessage(reply);
        history("message", created.getId(), "MESSAGE_REPLY_CREATED", actor(safe.getActor()), "replyTo=" + id);
        return created;
    }

    public ProductDtos.MessageSummary forwardMessage(String id, ProductDtos.MessageRequest request) {
        ProductDtos.MessageSummary original = message(id);
        ProductDtos.MessageRequest safe = request == null ? new ProductDtos.MessageRequest() : request;
        ProductDtos.MessageRequest forward = new ProductDtos.MessageRequest();
        forward.setMissionId(value(safe.getMissionId(), original.getMissionId()));
        forward.setReservationId(value(safe.getReservationId(), original.getReservationId()));
        forward.setFamily(value(safe.getFamily(), original.getFamily()));
        forward.setDirection(value(safe.getDirection(), "OUTBOUND"));
        forward.setSubject(value(safe.getSubject(), "FWD: " + value(original.getSubject(), original.getFamily())));
        forward.setRawText(blank(safe.getRawText()) ? original.getRawText() : safe.getRawText());
        forward.setActor(safe.getActor());
        ProductDtos.MessageSummary created = sendMessage(forward);
        history("message", created.getId(), "MESSAGE_FORWARD_CREATED", actor(safe.getActor()), "forwardOf=" + id);
        return created;
    }

    public List<ProductDtos.MessageSummary> messages() {
        if (usePersistence()) {
            return persistenceService.messages();
        }
        return new ArrayList<>(store.messages.values());
    }

    public ProductDtos.MessageSummary message(String id) {
        if (usePersistence()) {
            return persistenceService.message(id);
        }
        ProductDtos.MessageSummary message = store.messages.get(id);
        if (message == null) {
            throw new IllegalArgumentException("Unknown message: " + id);
        }
        return message;
    }

    public OperationalFeedBatchResult ingestFeed(ProductDtos.FeedIngestRequest request) {
        ProductDtos.FeedIngestRequest safe = request == null ? new ProductDtos.FeedIngestRequest() : request;
        OperationalFeedType type = type(safe.getType());
        OperationalFeedEnvelope envelope = OperationalFeedEnvelope.builder()
                .id(UUID.randomUUID().toString())
                .sourceId(value(safe.getSourceId(), "operator"))
                .type(type)
                .rawPayload(safe.getRawPayload())
                .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .build();
        OperationalFeedBatchResult batch = feedIngestService.ingest(new InMemoryOperationalFeedSource(envelope.getSourceId(),
                Collections.singletonList(envelope)).poll());
        for (OperationalFeedIngestResult result : batch.getResults()) {
            store.feedArtifacts.put(result.getEnvelope().getId(), result);
            if (usePersistence()) {
                persistenceService.saveFeedArtifact(result);
            }
            history("feed", result.getEnvelope().getId(), "FEED_INGESTED", "system",
                    result.isAccepted() ? "accepted" : "rejected");
        }
        return batch;
    }

    public List<ProductDtos.FeedArtifactSummary> feedArtifacts() {
        if (usePersistence()) {
            return persistenceService.feedArtifacts();
        }
        List<ProductDtos.FeedArtifactSummary> out = new ArrayList<>();
        synchronized (store.feedArtifacts) {
            for (OperationalFeedIngestResult result : store.feedArtifacts.values()) {
                List<String> diagnostics = new ArrayList<>(result.getWarnings());
                diagnostics.addAll(result.getErrors());
                out.add(ProductDtos.FeedArtifactSummary.builder()
                        .id(result.getEnvelope().getId())
                        .sourceId(result.getEnvelope().getSourceId())
                        .type(result.getEnvelope().getType().name())
                        .accepted(result.isAccepted())
                        .rawPayloadHash(result.getRawPayloadHash())
                        .rawPayload(result.getEnvelope().getRawPayload())
                        .receivedAt(result.getEnvelope().getReceivedAt())
                        .downstreamArtifactIds(new ArrayList<>(result.getDownstreamArtifactIds()))
                        .diagnostics(diagnostics)
                        .build());
            }
        }
        return out;
    }

    public ProductDtos.FeedArtifactSummary feedArtifact(String id) {
        if (usePersistence()) {
            return persistenceService.feedArtifact(id);
        }
        OperationalFeedIngestResult result = store.feedArtifacts.get(id);
        if (result == null) {
            throw new IllegalArgumentException("Unknown feed artifact: " + id);
        }
        List<String> diagnostics = new ArrayList<>(result.getWarnings());
        diagnostics.addAll(result.getErrors());
        return ProductDtos.FeedArtifactSummary.builder()
                .id(result.getEnvelope().getId())
                .sourceId(result.getEnvelope().getSourceId())
                .type(result.getEnvelope().getType().name())
                .accepted(result.isAccepted())
                .rawPayloadHash(result.getRawPayloadHash())
                .rawPayload(result.getEnvelope().getRawPayload())
                .receivedAt(result.getEnvelope().getReceivedAt())
                .downstreamArtifactIds(new ArrayList<>(result.getDownstreamArtifactIds()))
                .diagnostics(diagnostics)
                .build();
    }

    public List<ProductDtos.FeedTransactionSummary> feedTransactions(String id) {
        OperationalFeedIngestResult result = store.feedArtifacts.get(id);
        if (result == null && usePersistence()) {
            ProductDtos.FeedArtifactSummary artifact = persistenceService.feedArtifact(id);
            OperationalFeedEnvelope envelope = OperationalFeedEnvelope.builder()
                    .id(artifact.getId())
                    .sourceId(artifact.getSourceId())
                    .type(type(artifact.getType()))
                    .rawPayload(artifact.getRawPayload())
                    .receivedAt(artifact.getReceivedAt())
                    .build();
            result = feedIngestService.ingest(new InMemoryOperationalFeedSource(envelope.getSourceId(),
                    Collections.singletonList(envelope)).poll()).getResults().get(0);
        }
        if (result == null) {
            throw new IllegalArgumentException("Unknown feed artifact: " + id);
        }
        List<ProductDtos.FeedTransactionSummary> values = new ArrayList<>();
        if (result.getUsnsResult() != null && result.getUsnsResult().getTransactionIngestResults() != null) {
            for (org.tash.extensions.messaging.UsnsTransactionIngestResult transactionResult
                    : result.getUsnsResult().getTransactionIngestResults()) {
                values.add(ProductDtos.FeedTransactionSummary.builder()
                        .id(id + "#" + values.size())
                        .type(transactionResult.getTransaction() == null || transactionResult.getTransaction().getType() == null
                                ? "UNKNOWN" : transactionResult.getTransaction().getType().name())
                        .status(transactionResult.getRoutingOutcome() == null ? (transactionResult.isSupported() ? "SUPPORTED" : "CLASSIFIED_ONLY")
                                : String.valueOf(transactionResult.getRoutingOutcome().getType()))
                        .supported(transactionResult.isSupported())
                        .normalizedText(transactionResult.getTransaction() == null ? null : transactionResult.getTransaction().getNormalizedText())
                        .warnings(transactionResult.getWarnings() == null ? Collections.emptyList() : transactionResult.getWarnings())
                        .errors(transactionResult.getErrors() == null ? Collections.emptyList() : transactionResult.getErrors())
                        .build());
            }
        }
        if (values.isEmpty()) {
            values.add(ProductDtos.FeedTransactionSummary.builder()
                    .id(id)
                    .type(result.getEnvelope().getType() == null ? "UNKNOWN" : result.getEnvelope().getType().name())
                    .status(result.isAccepted() ? "ACCEPTED" : "REJECTED")
                    .supported(result.isAccepted())
                    .normalizedText(result.getEnvelope().getRawPayload())
                    .warnings(new ArrayList<>(result.getWarnings()))
                    .errors(new ArrayList<>(result.getErrors()))
                    .build());
        }
        return values;
    }

    public ProductDtos.DecisionSummary evaluateDecision(ProductDtos.DecisionEvaluateRequest request) {
        ProductDtos.DecisionEvaluateRequest safe = request == null ? new ProductDtos.DecisionEvaluateRequest() : request;
        OperationalDecisionRequest engineRequest = OperationalDecisionRequest.builder()
                .decisionTime(parseTime(safe.getDecisionTime()))
                .rawUsnsMessages(safe.getRawUsnsMessages())
                .rawCarfMessages(safe.getRawCarfMessages())
                .route(route(safe.getRoute()))
                .build();
        OperationalDecisionResult result = decisionEngine.evaluate(engineRequest);
        String decisionId = UUID.randomUUID().toString();
        if (usePersistence()) {
            decisionId = persistenceService.saveDecision(result).toString();
        }
        ProductDtos.DecisionSummary summary = ProductDtos.DecisionSummary.builder()
                .id(decisionId)
                .action(result.getAction() == null ? null : result.getAction().name())
                .recommendedAction(result.getRecommendedAction() == null ? null : result.getRecommendedAction().name())
                .confidence(result.getConfidence())
                .rationale(result.getRationale())
                .resultJson(org.tash.extensions.engine.CanonicalJson.write(result))
                .auditJson(result.getAuditEnvelope() == null ? null : org.tash.extensions.engine.CanonicalJson.write(result.getAuditEnvelope()))
                .replayJson(result.getReplayBundle() == null ? null : org.tash.extensions.engine.CanonicalJson.write(result.getReplayBundle()))
                .result(result)
                .build();
        store.decisions.put(summary.getId(), summary);
        history("decision", summary.getId(), "DECISION_EVALUATED", "system", summary.getAction());
        return summary;
    }

    public ProductDtos.DecisionSummary decision(String id) {
        ProductDtos.DecisionSummary decision = store.decisions.get(id);
        if (decision == null && usePersistence()) {
            return persistenceService.decision(id);
        }
        if (decision == null) {
            throw new IllegalArgumentException("Unknown decision: " + id);
        }
        return decision;
    }

    public OperationalDecisionResult decisionResult(String id) {
        ProductDtos.DecisionSummary decision = store.decisions.get(id);
        if (decision != null && decision.getResult() != null) {
            return decision.getResult();
        }
        if (usePersistence()) {
            return persistenceService.decisionResult(id);
        }
        throw new IllegalArgumentException("Unknown decision: " + id);
    }

    public OperationalDecisionRequest decisionRequest(String id) {
        ProductDtos.DecisionSummary decision = store.decisions.get(id);
        if (decision != null && decision.getResult() != null && decision.getResult().getReplayBundle() != null) {
            return decision.getResult().getReplayBundle().getRequest();
        }
        if (usePersistence()) {
            return persistenceService.decisionRequest(id);
        }
        throw new IllegalArgumentException("Unknown decision request: " + id);
    }

    public OperationalDecisionResult rehydrateDecision(String id) {
        return decisionResult(id);
    }

    public ProductDtos.ReservationSummary rehydrateReservation(String id) {
        if (usePersistence()) {
            return persistenceService.rehydrateReservation(id);
        }
        ReservationWorkflowRecord record = store.reservations.get(id);
        if (record == null) {
            throw new IllegalArgumentException("Unknown reservation: " + id);
        }
        return reservationSummary(store.reservationMissionIds.get(id), record);
    }

    public OperationalDecisionReplayBundle decisionReplayBundle(String id) {
        ProductDtos.DecisionSummary decision = store.decisions.get(id);
        if (decision != null && decision.getResult() != null && decision.getResult().getReplayBundle() != null) {
            return decision.getResult().getReplayBundle();
        }
        if (usePersistence()) {
            return persistenceService.decisionReplayBundle(id);
        }
        throw new IllegalArgumentException("Unknown decision replay bundle: " + id);
    }

    public OperationalDecisionAuditEnvelope decisionAuditEnvelope(String id) {
        ProductDtos.DecisionSummary decision = store.decisions.get(id);
        if (decision != null && decision.getResult() != null && decision.getResult().getAuditEnvelope() != null) {
            return decision.getResult().getAuditEnvelope();
        }
        if (usePersistence()) {
            return persistenceService.decisionAuditEnvelope(id);
        }
        throw new IllegalArgumentException("Unknown decision audit envelope: " + id);
    }

    public List<ProductDtos.HistoryEventSummary> history() {
        if (usePersistence()) {
            return persistenceService.history();
        }
        return new ArrayList<>(store.history.values());
    }

    public List<ProductDtos.SearchResultSummary> search(String query) {
        if (usePersistence()) {
            return persistenceService.search(query);
        }
        String needle = normalize(query);
        List<ProductDtos.SearchResultSummary> results = new ArrayList<>();
        for (ProductDtos.MissionSummary mission : store.missions.values()) {
            if (matches(needle, mission.getMissionNumber(), mission.getTitle(), mission.getStatus())) {
                results.add(ProductDtos.SearchResultSummary.builder()
                        .id(mission.getId())
                        .type("mission")
                        .title(value(mission.getMissionNumber(), mission.getId()))
                        .status(mission.getStatus())
                        .snippet(value(mission.getTitle(), "Mission"))
                        .route("/missions/" + mission.getId())
                        .updatedAt(mission.getUpdatedAt())
                        .build());
            }
        }
        for (ProductDtos.MessageSummary message : store.messages.values()) {
            if (matches(needle, message.getSubject(), message.getRawText(), message.getFamily(), message.getDirection(), message.getStatus())) {
                results.add(ProductDtos.SearchResultSummary.builder()
                        .id(message.getId())
                        .type("message")
                        .title(value(message.getSubject(), message.getFamily()))
                        .status(message.getStatus())
                        .snippet(snippet(message.getRawText()))
                        .route("/messages/" + message.getId())
                        .updatedAt(message.getCreatedAt())
                        .build());
            }
        }
        for (ProductDtos.FeedArtifactSummary artifact : feedArtifacts()) {
            if (matches(needle, artifact.getSourceId(), artifact.getType(), artifact.getRawPayload(), artifact.getRawPayloadHash())) {
                results.add(ProductDtos.SearchResultSummary.builder()
                        .id(artifact.getId())
                        .type("feed")
                        .title(artifact.getSourceId() + " " + artifact.getType())
                        .status(artifact.isAccepted() ? "ACCEPTED" : "REJECTED")
                        .snippet(snippet(artifact.getRawPayload()))
                        .route("/feed/" + artifact.getId())
                        .updatedAt(artifact.getReceivedAt())
                        .build());
            }
        }
        for (ProductDtos.DecisionSummary decision : store.decisions.values()) {
            if (matches(needle, decision.getAction(), decision.getRecommendedAction(), decision.getRationale(), decision.getResultJson())) {
                results.add(ProductDtos.SearchResultSummary.builder()
                        .id(decision.getId())
                        .type("decision")
                        .title(value(decision.getAction(), "Decision"))
                        .status(decision.getRecommendedAction())
                        .snippet(snippet(decision.getRationale()))
                        .route("/decisions/" + decision.getId())
                        .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                        .build());
            }
        }
        for (ProductDtos.HistoryEventSummary event : store.history.values()) {
            if (matches(needle, event.getAggregateType(), event.getAggregateId(), event.getEventType(), event.getActor(), event.getNote())) {
                results.add(ProductDtos.SearchResultSummary.builder()
                        .id(event.getId())
                        .type("history")
                        .title(event.getEventType())
                        .status(event.getAggregateType())
                        .snippet(event.getNote())
                        .route("/history")
                        .updatedAt(event.getCreatedAt())
                        .build());
            }
        }
        return results;
    }

    public List<ProductDtos.ReferencePointSummary> referencePoints(String pointType) {
        if (usePersistence()) {
            List<ProductDtos.ReferencePointSummary> persisted = persistenceService.referencePoints(pointType);
            if (!persisted.isEmpty()) {
                return persisted;
            }
        }
        String type = blank(pointType) ? "" : pointType.trim().toUpperCase(Locale.US);
        List<ProductDtos.ReferencePointSummary> points = new ArrayList<>();
        synchronized (store.referencePoints) {
            for (ProductDtos.ReferencePointSummary point : store.referencePoints.values()) {
                if (type.isEmpty() || type.equals(point.getPointType())) {
                    points.add(point);
                }
            }
        }
        for (ProductDtos.ReferencePointSummary point : seedReferencePoints().values()) {
            if (type.isEmpty() || type.equals(point.getPointType())) {
                points.add(point);
            }
        }
        return points;
    }

    public ProductDtos.ReferencePointSummary createReferencePoint(ProductDtos.ReferencePointRequest request) {
        ProductDtos.ReferencePointRequest safe = request == null ? new ProductDtos.ReferencePointRequest() : request;
        if (blank(safe.getIdentifier())) {
            throw new IllegalArgumentException("Reference point identifier is required");
        }
        if (blank(safe.getPointType())) {
            safe.setPointType("FIX");
        }
        if (usePersistence()) {
            return persistenceService.saveReferencePoint(safe);
        }
        ProductDtos.ReferencePointSummary summary = ProductDtos.ReferencePointSummary.builder()
                .id(UUID.nameUUIDFromBytes((safe.getPointType() + ":" + safe.getIdentifier()).getBytes(java.nio.charset.StandardCharsets.UTF_8)).toString())
                .identifier(safe.getIdentifier().trim().toUpperCase(Locale.US))
                .pointType(safe.getPointType().trim().toUpperCase(Locale.US))
                .latitude(safe.getLatitude())
                .longitude(safe.getLongitude())
                .altitudeFeet(safe.getAltitudeFeet())
                .source(value(safe.getSource(), "operator"))
                .metadataJson(safe.getMetadataJson())
                .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .build();
        store.referencePoints.put(summary.getIdentifier(), summary);
        history("reference", summary.getId(), "REFERENCE_POINT_CREATED", "system", summary.getIdentifier());
        return summary;
    }

    public List<ProductDtos.ReservationSupplementSummary> supplements(String reservationId) {
        if (usePersistence()) {
            return persistenceService.supplements(reservationId);
        }
        List<ProductDtos.ReservationSupplementSummary> values = new ArrayList<>();
        synchronized (store.supplements) {
            for (ProductDtos.ReservationSupplementSummary supplement : store.supplements.values()) {
                if (reservationId.equals(supplement.getReservationId())) {
                    values.add(supplement);
                }
            }
        }
        return values;
    }

    public ProductDtos.ReservationSupplementSummary createSupplement(String reservationId,
                                                                     ProductDtos.ReservationSupplementRequest request) {
        ProductDtos.ReservationSupplementRequest safe = request == null ? new ProductDtos.ReservationSupplementRequest() : request;
        if (blank(safe.getKind())) {
            safe.setKind("COORDINATION");
        }
        if (usePersistence()) {
            ProductDtos.ReservationSupplementSummary saved = persistenceService.saveSupplement(reservationId, safe);
            history("reservation", reservationId, "SUPPLEMENT_CREATED", actor(safe.getActor()), saved.getKind());
            return saved;
        }
        ProductDtos.ReservationSupplementSummary summary = ProductDtos.ReservationSupplementSummary.builder()
                .id(UUID.randomUUID().toString())
                .reservationId(reservationId)
                .kind(safe.getKind().trim().toUpperCase(Locale.US))
                .status(value(safe.getStatus(), "OPEN"))
                .title(value(safe.getTitle(), safe.getKind()))
                .text(safe.getText())
                .actor(actor(safe.getActor()))
                .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .build();
        store.supplements.put(summary.getId(), summary);
        history("reservation", reservationId, "SUPPLEMENT_CREATED", summary.getActor(), summary.getKind());
        return summary;
    }

    public ProductDtos.ReservationSupplementSummary transitionSupplement(String supplementId,
                                                                         ProductDtos.SupplementTransitionRequest request) {
        ProductDtos.SupplementTransitionRequest safe = request == null ? new ProductDtos.SupplementTransitionRequest() : request;
        String status = value(safe.getStatus(), "OPEN").trim().toUpperCase(Locale.US);
        if (usePersistence()) {
            ProductDtos.ReservationSupplementSummary transitioned = persistenceService.transitionSupplement(supplementId, safe);
            history("reservation", transitioned.getReservationId(), "SUPPLEMENT_" + status,
                    actor(safe.getActor()), value(safe.getNote(), transitioned.getKind()));
            return transitioned;
        }
        ProductDtos.ReservationSupplementSummary existing = store.supplements.get(supplementId);
        if (existing == null) {
            throw new IllegalArgumentException("Unknown reservation supplement: " + supplementId);
        }
        ProductDtos.ReservationSupplementSummary transitioned = existing.toBuilder()
                .status(status)
                .actor(actor(safe.getActor()))
                .text(blank(safe.getNote()) ? existing.getText() : safe.getNote())
                .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .build();
        store.supplements.put(supplementId, transitioned);
        history("reservation", transitioned.getReservationId(), "SUPPLEMENT_" + status,
                transitioned.getActor(), transitioned.getKind());
        return transitioned;
    }

    public ProductDtos.ReferenceDataImportResult previewReferenceData(ProductDtos.ReferenceDataImportRequest request) {
        ProductDtos.ReferenceDataImportRequest safe = request == null ? new ProductDtos.ReferenceDataImportRequest() : request;
        ReferenceDataSyncResult sync = new LocalReferenceDataSyncAdapter().preview(safe.getPayload());
        return referenceDataResult(sync, false);
    }

    public ProductDtos.ReferenceDataImportResult applyReferenceData(ProductDtos.ReferenceDataImportRequest request) {
        ProductDtos.ReferenceDataImportRequest safe = request == null ? new ProductDtos.ReferenceDataImportRequest() : request;
        ReferenceDataSyncResult sync = new LocalReferenceDataSyncAdapter().preview(safe.getPayload());
        if (!sync.isAccepted()) {
            return referenceDataResult(sync, false);
        }
        List<ProductDtos.ReferencePointSummary> points = new ArrayList<>();
        for (ReferenceDataImportRecord record : sync.getRecords()) {
            ProductDtos.ReferencePointRequest point = new ProductDtos.ReferencePointRequest();
            point.setIdentifier(record.getIdentifier());
            point.setPointType(record.getPointType());
            point.setLatitude(record.getLatitude());
            point.setLongitude(record.getLongitude());
            point.setAltitudeFeet(record.getAltitudeFeet());
            point.setSource(record.getSource());
            point.setMetadataJson(record.getMetadataJson());
            points.add(createReferencePoint(point));
        }
        ProductDtos.ReferenceDataImportResult result = referenceDataResult(sync, true);
        result.setAppliedCount(points.size());
        result.setPoints(points);
        history("reference", "bulk-import", "REFERENCE_DATA_SYNC_APPLIED", actor(safe.getActor()),
                "records=" + points.size());
        return result;
    }

    public Map<String, GeoCoordinate> referenceMap(String pointType) {
        Map<String, GeoCoordinate> values = new LinkedHashMap<>();
        for (ProductDtos.ReferencePointSummary point : referencePoints(pointType)) {
            values.put(point.getIdentifier(), GeoCoordinate.builder()
                    .latitude(point.getLatitude())
                    .longitude(point.getLongitude())
                    .altitude(point.getAltitudeFeet() == null ? 0.0 : point.getAltitudeFeet())
                    .build());
        }
        return values;
    }

    public java.util.Map<String, Double> metrics() {
        if (usePersistence()) {
            return persistenceService.metrics();
        }
        Map<String, Double> values = new LinkedHashMap<>();
        List<ProductDtos.FeedArtifactSummary> feeds = feedArtifacts();
        values.put("product.missions", (double) missions().size());
        values.put("product.reservations", (double) store.reservations.size());
        values.put("product.messages", (double) messages().size());
        values.put("product.feedArtifacts", (double) feeds.size());
        values.put("product.feedAccepted", (double) feeds.stream().filter(ProductDtos.FeedArtifactSummary::isAccepted).count());
        values.put("product.feedRejected", (double) feeds.stream().filter(feed -> !feed.isAccepted()).count());
        values.put("product.decisions", (double) store.decisions.size());
        values.put("product.historyEvents", (double) history().size());
        values.put("product.referencePoints", (double) referencePoints(null).size());
        values.put("product.supplements", (double) store.supplements.size());
        values.put("product.records", values.values().stream().mapToDouble(Double::doubleValue).sum());
        return values;
    }

    private ProductDtos.MissionSummary requireMission(String id) {
        if (usePersistence()) {
            return persistenceService.mission(id).getMission();
        }
        ProductDtos.MissionSummary mission = store.missions.get(id);
        if (mission == null) {
            throw new IllegalArgumentException("Unknown mission: " + id);
        }
        return mission;
    }

    private ReservationWorkflowResult recordReservationWorkflowResult(String reservationId,
                                                                      ReservationWorkflowResult result,
                                                                      String eventType,
                                                                      String actor,
                                                                      String note) {
        if (result != null && result.getRecord() != null) {
            store.reservations.put(reservationId, result.getRecord());
            if (usePersistence()) {
                persistenceService.saveReservation(result.getRecord());
            }
            history("reservation", reservationId, eventType, actor, note == null ? result.getRecord().getState().name() : note);
        }
        return result;
    }

    private ProductDtos.ReservationSummary reservationSummary(String missionId, ReservationWorkflowRecord record) {
        return ProductDtos.ReservationSummary.builder()
                .id(record.getId())
                .missionId(missionId)
                .state(record.getState())
                .rawText(record.getDraft() == null ? null : record.getDraft().getRawText())
                .lockedBy(record.getLockOwner())
                .lockedAt(record.getLockedAt())
                .conflictCount(record.getConflictReviews() == null ? 0 : record.getConflictReviews().size())
                .diagnostics(record.getDiagnostics())
                .build();
    }

    private OperationalFeedType type(String value) {
        if (value == null) {
            return OperationalFeedType.UNKNOWN;
        }
        return OperationalFeedType.valueOf(value.trim().toUpperCase(Locale.US));
    }

    private ZonedDateTime parseTime(String value) {
        return blank(value) ? ZonedDateTime.parse("2026-05-20T00:00:00Z") : ZonedDateTime.parse(value);
    }

    private boolean usePersistence() {
        return persistenceEnabled && persistenceService != null;
    }

    private ProductDtos.HistoryEventSummary history(String aggregateType, String aggregateId,
                                                    String eventType, String actor, String note) {
        ProductDtos.HistoryEventSummary event = store.history(aggregateType, aggregateId, eventType, actor, note);
        if (usePersistence()) {
            persistenceService.saveHistory(event);
        }
        return event;
    }

    private List<GeoCoordinate> route(List<List<Double>> values) {
        List<GeoCoordinate> points = new ArrayList<>();
        if (values != null) {
            for (List<Double> value : values) {
                if (value != null && value.size() >= 2) {
                    points.add(GeoCoordinate.builder()
                            .latitude(value.get(0))
                            .longitude(value.get(1))
                            .altitude(value.size() > 2 ? value.get(2) : 0.0)
                            .build());
                }
            }
        }
        return points;
    }

    private String value(String value, String fallback) {
        return blank(value) ? fallback : value;
    }

    private boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private String actor(String value) {
        return blank(value) ? "system" : value;
    }

    private boolean matches(String needle, String... values) {
        if (blank(needle)) {
            return true;
        }
        for (String value : values) {
            if (value != null && value.toLowerCase(Locale.US).contains(needle)) {
                return true;
            }
        }
        return false;
    }

    private String normalize(String query) {
        return query == null ? "" : query.trim().toLowerCase(Locale.US);
    }

    private String snippet(String value) {
        if (value == null) {
            return "";
        }
        String compact = value.replaceAll("\\s+", " ").trim();
        return compact.length() <= 160 ? compact : compact.substring(0, 157) + "...";
    }

    private Map<String, ProductDtos.ReferencePointSummary> seedReferencePoints() {
        Map<String, ProductDtos.ReferencePointSummary> values = new LinkedHashMap<>();
        values.put("JFK", referencePoint("JFK", "NAVAID", 40.6398, -73.7789, 13.0, "seed"));
        values.put("DOV", referencePoint("DOV", "NAVAID", 39.1295, -75.4660, 24.0, "seed"));
        values.put("3000N15000W", referencePoint("3000N15000W", "FIX", 30.0, -150.0, 0.0, "seed"));
        values.put("3100N14900W", referencePoint("3100N14900W", "FIX", 31.0, -149.0, 0.0, "seed"));
        return values;
    }

    private ProductDtos.ReferencePointSummary referencePoint(String identifier, String type, double latitude,
                                                            double longitude, Double altitudeFeet, String source) {
        return ProductDtos.ReferencePointSummary.builder()
                .id(UUID.nameUUIDFromBytes((type + ":" + identifier).getBytes(java.nio.charset.StandardCharsets.UTF_8)).toString())
                .identifier(identifier)
                .pointType(type)
                .latitude(latitude)
                .longitude(longitude)
                .altitudeFeet(altitudeFeet)
                .source(source)
                .updatedAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                .build();
    }

    private ProductDtos.ReferenceDataImportResult referenceDataResult(ReferenceDataSyncResult sync, boolean applied) {
        List<ProductDtos.ReferencePointSummary> points = new ArrayList<>();
        for (ReferenceDataImportRecord record : sync.getRecords()) {
            points.add(ProductDtos.ReferencePointSummary.builder()
                    .id(UUID.nameUUIDFromBytes((record.getPointType() + ":" + record.getIdentifier()).getBytes(java.nio.charset.StandardCharsets.UTF_8)).toString())
                    .identifier(record.getIdentifier())
                    .pointType(record.getPointType())
                    .latitude(record.getLatitude())
                    .longitude(record.getLongitude())
                    .altitudeFeet(record.getAltitudeFeet())
                    .source(record.getSource())
                    .metadataJson(record.getMetadataJson())
                    .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                    .build());
        }
        return ProductDtos.ReferenceDataImportResult.builder()
                .accepted(sync.isAccepted())
                .parsedCount(sync.getRecords().size())
                .appliedCount(applied ? points.size() : 0)
                .warnings(new ArrayList<>(sync.getWarnings()))
                .errors(new ArrayList<>(sync.getErrors()))
                .points(points)
                .build();
    }
}
