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
import org.tash.extensions.engine.OperationalConstraint;
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
import org.tash.extensions.routing.RouteCandidate;
import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.pirep.PirepReport;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductParser;
import org.tash.extensions.workflow.ReservationWorkflowRecord;
import org.tash.extensions.workflow.ReservationWorkflowResult;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.time.Duration;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.UUID;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@ApplicationScoped
public class AirspaceProductService {
    private final ProductRecordStore store = new ProductRecordStore();
    private final ReservationWorkflowService workflowService;
    private final ProductPersistenceService persistenceService;
    private final boolean persistenceEnabled;
    private final OperationalFeedIngestService feedIngestService = new OperationalFeedIngestService();
    private final OperationalDecisionEngine decisionEngine = new OperationalDecisionEngine();
    private final WeatherProductParser weatherProductParser = new WeatherProductParser();

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

    public ProductDtos.MissionWeatherVerdictSummary missionWeatherVerdict(String missionId) {
        requireMission(missionId);
        List<ProductDtos.MessageSummary> weather = guidanceMessagesForMission(missionId);
        List<ProductDtos.WeatherSourceSummary> sources = new ArrayList<>();
        List<String> diagnostics = new ArrayList<>();
        for (ProductDtos.MessageSummary message : weather) {
            sources.add(sourceSummary(message, guidanceAction(message), guidanceRationale(message)));
        }
        ProductDtos.RouteImpactSummary impact = routeImpact(missionId, null);
        String action = strongestAction(impact, sources);
        boolean stale = sources.stream().anyMatch(ProductDtos.WeatherSourceSummary::isStale);
        double confidence = Math.max(0.35, Math.min(0.97,
                impact.getConfidence() > 0.0 ? impact.getConfidence() : sources.isEmpty() ? 0.95 : 0.78));
        if (stale) {
            confidence = Math.max(0.35, confidence - 0.08);
            diagnostics.add("One or more weather/PIREP products are stale relative to the local decision clock");
        }
        return ProductDtos.MissionWeatherVerdictSummary.builder()
                .missionId(missionId)
                .action(action)
                .priority(priorityFor(action, sources))
                .confidence(confidence)
                .sourceCount(sources.size())
                .stale(stale)
                .summary(sources.isEmpty()
                        ? "No active weather, PIREP, or NOTAM source artifacts are linked to this mission."
                        : sources.size() + " source artifact(s); " + impact.getRationale())
                .recommendedAction(recommendedActionFor(action))
                .sources(sources)
                .diagnostics(diagnostics)
                .build();
    }

    public List<ProductDtos.WeatherSourceSummary> weatherChanges(String missionId, String since, Integer limit) {
        requireMission(missionId);
        ZonedDateTime sinceTime = blank(since) ? ZonedDateTime.now(ZoneOffset.UTC).minusHours(4) : parseTime(since);
        int max = limit == null || limit <= 0 ? 12 : Math.min(limit, 100);
        List<ProductDtos.WeatherSourceSummary> changes = new ArrayList<>();
        for (ProductDtos.MessageSummary message : guidanceMessagesForMission(missionId)) {
            ProductDtos.WeatherSourceSummary source = sourceSummary(message, guidanceAction(message), guidanceRationale(message));
            if (source.getObservedAt() == null || !source.getObservedAt().isBefore(sinceTime)) {
                changes.add(source);
            }
        }
        changes.sort((left, right) -> {
            int byTime = String.valueOf(right.getObservedAt()).compareTo(String.valueOf(left.getObservedAt()));
            if (byTime != 0) {
                return byTime;
            }
            return severityRank(right.getSeverity()) - severityRank(left.getSeverity());
        });
        return changes.size() > max ? new ArrayList<>(changes.subList(0, max)) : changes;
    }

    public ProductDtos.RouteImpactSummary routeImpact(String missionId, String reservationId) {
        ProductDtos.MissionDetail detail = mission(missionId);
        List<GeoCoordinate> route = routeForMission(detail, reservationId);
        ParsedWeatherInputs inputs = parsedWeatherInputs(weatherMessagesForMission(missionId));
        OperationalDecisionResult decision = decisionEngine.evaluate(OperationalDecisionRequest.builder()
                .decisionTime(ZonedDateTime.now(ZoneOffset.UTC))
                .weatherProducts(inputs.products)
                .pireps(inputs.pireps)
                .route(route)
                .build());
        List<String> impacted = new ArrayList<>();
        List<String> sourceRefs = new ArrayList<>();
        int segments = 0;
        int notamConstraints = 0;
        for (RouteBlockagePrediction blockage : decision.getRouteBlockages()) {
            if (blockage.getPrimaryHazardId() != null) {
                sourceRefs.add(blockage.getPrimaryHazardId());
            }
            for (Integer index : blockage.getBlockedSegmentIndexes()) {
                impacted.add("segment " + index + " forecast-hour " + blockage.getForecastHour()
                        + " probability " + Math.round(blockage.getBlockedProbability() * 100.0) + "%");
            }
            segments += blockage.getBlockedSegmentIndexes().size();
        }
        for (OperationalConstraint constraint : decision.getBlockingConstraints()) {
            constraint.getSources().forEach(source -> sourceRefs.add(source.getType() + ":" + source.getId()));
        }
        for (ProductDtos.MessageSummary message : guidanceMessagesForMission(missionId)) {
            if (!isNotamTraffic(message.getFamily(), message.getRawText(), message.getSubject())) {
                continue;
            }
            if (reservationId != null && message.getReservationId() != null && !reservationId.equals(message.getReservationId())) {
                continue;
            }
            notamConstraints++;
            sourceRefs.add(value(message.getFamily(), "NOTAM") + ":" + message.getId());
            impacted.add("NOTAM constraint " + value(message.getSubject(), snippet(message.getRawText()))
                    + " requires route/altitude/time review");
        }
        List<String> candidates = new ArrayList<>();
        if (decision.getRoutePlanResult() != null) {
            for (RouteCandidate candidate : decision.getRoutePlanResult().getCandidates()) {
                candidates.add(candidate.getId() + " avoids " + candidate.getAvoidedConstraintIds()
                        + " cost " + Math.round(candidate.getCost()));
            }
        }
        return ProductDtos.RouteImpactSummary.builder()
                .missionId(missionId)
                .reservationId(reservationId)
                .action(decision.getAction() == null ? "CLEAR" : decision.getAction().name())
                .recommendedAction(decision.getRecommendedAction() == null ? "NONE" : decision.getRecommendedAction().name())
                .confidence(decision.getConfidence())
                .rationale(routeImpactRationale(decision, notamConstraints))
                .impactedSegmentCount(segments + notamConstraints)
                .blockingConstraintCount(decision.getBlockingConstraints().size() + notamConstraints)
                .impactedSegments(impacted)
                .sourceRefs(distinct(sourceRefs))
                .avoidanceCandidates(candidates)
                .diagnostics(inputs.diagnostics)
                .build();
    }

    public ProductDtos.PirepRelevanceResult relevantPireps(String missionId, ProductDtos.PirepRelevanceRequest request) {
        ProductDtos.PirepRelevanceRequest safe = request == null ? new ProductDtos.PirepRelevanceRequest() : request;
        List<GeoCoordinate> route = safe.getRoute() == null || safe.getRoute().isEmpty()
                ? routeForMission(mission(missionId), safe.getReservationId())
                : route(safe.getRoute());
        double lower = safe.getLowerAltitudeFeet() == null ? 0.0 : safe.getLowerAltitudeFeet();
        double upper = safe.getUpperAltitudeFeet() == null ? 60000.0 : safe.getUpperAltitudeFeet();
        double altitudeTolerance = safe.getAltitudeToleranceFeet() == null ? 2000.0 : Math.max(0.0, safe.getAltitudeToleranceFeet());
        int recency = safe.getRecencyMinutes() == null ? 60 : safe.getRecencyMinutes();
        double corridor = safe.getCorridorNauticalMiles() == null ? 50.0 : safe.getCorridorNauticalMiles();
        ZonedDateTime now = ZonedDateTime.now(ZoneOffset.UTC);
        ParsedWeatherInputs inputs = parsedWeatherInputs(weatherMessagesForMission(missionId));
        List<ProductDtos.WeatherSourceSummary> relevant = new ArrayList<>();
        List<ProductDtos.WeatherSourceSummary> excluded = new ArrayList<>();
        double scoreTotal = 0.0;
        int staleCount = 0;
        for (PirepReport report : inputs.pireps) {
            boolean altitudeMatch = report.getAltitudeFeet() == null
                    || (report.getAltitudeFeet() >= lower - altitudeTolerance && report.getAltitudeFeet() <= upper + altitudeTolerance);
            long ageMinutes = ageMinutes(report.getObservationTime(), now);
            boolean timeMatch = report.getObservationTime() == null
                    || !report.getObservationTime().isBefore(now.minusMinutes(recency));
            boolean routeMatch = report.getLocation() == null || route.isEmpty()
                    || distanceToRoute(report.getLocation(), route) <= corridor;
            double relevanceScore = pirepRelevanceScore(altitudeMatch, timeMatch, routeMatch, ageMinutes, recency);
            String agingCategory = agingCategory(ageMinutes, recency);
            boolean stale = !timeMatch || "STALE".equals(agingCategory);
            if (stale) {
                staleCount++;
            }
            scoreTotal += relevanceScore;
            ProductDtos.WeatherSourceSummary summary = ProductDtos.WeatherSourceSummary.builder()
                    .id(value(report.getId(), "PIREP"))
                    .family("PIREP")
                    .label(value(report.getLocationText(), value(report.getAircraftType(), "PIREP")))
                    .route("/weather")
                    .severity(report.isUrgent() ? "SEVERE" : String.valueOf(report.getIntensity()))
                    .rationale("PIREP " + value(report.getPhenomenon() == null ? null : report.getPhenomenon().name(), "WEATHER")
                            + " at " + (report.getAltitudeFeet() == null ? "unknown altitude" : Math.round(report.getAltitudeFeet()) + " ft")
                            + "; relevance " + Math.round(relevanceScore * 100.0) + "%")
                    .observedAt(report.getObservationTime())
                    .ageMinutes(ageMinutes)
                    .agingCategory(agingCategory)
                    .relevanceScore(relevanceScore)
                    .stale(stale)
                    .build();
            if (altitudeMatch && timeMatch && routeMatch) {
                relevant.add(summary);
            } else {
                excluded.add(summary);
            }
        }
        return ProductDtos.PirepRelevanceResult.builder()
                .missionId(missionId)
                .totalPireps(inputs.pireps.size())
                .relevantCount(relevant.size())
                .staleCount(staleCount)
                .averageRelevanceScore(inputs.pireps.isEmpty() ? 0.0 : scoreTotal / inputs.pireps.size())
                .altitudeToleranceFeet(altitudeTolerance)
                .recencyMinutes(recency)
                .corridorNauticalMiles(corridor)
                .relevant(relevant)
                .excluded(excluded)
                .build();
    }

    public ProductDtos.CoordinationDraftSummary coordinationDraft(String missionId,
                                                                  ProductDtos.CoordinationDraftRequest request) {
        ProductDtos.CoordinationDraftRequest safe = request == null ? new ProductDtos.CoordinationDraftRequest() : request;
        ProductDtos.MissionSummary mission = requireMission(missionId);
        ProductDtos.MissionWeatherVerdictSummary verdict = missionWeatherVerdict(missionId);
        ProductDtos.RouteImpactSummary impact = routeImpact(missionId, safe.getReservationId());
        List<String> refs = new ArrayList<>(impact.getSourceRefs());
        verdict.getSources().forEach(source -> refs.add(source.getFamily() + ":" + source.getId()));
        String subject = "WX COORD " + mission.getMissionNumber() + " " + verdict.getAction();
        String body = "USNS WEATHER COORDINATION\n"
                + "MISSION: " + mission.getMissionNumber() + "\n"
                + "ACTION: " + verdict.getAction() + "\n"
                + "RECOMMENDED: " + verdict.getRecommendedAction() + "\n"
                + "IMPACT: " + impact.getRationale() + "\n"
                + "SOURCES: " + String.join(", ", distinct(refs)) + "\n"
                + "REQUEST: WEATHER DESK / TRAFFIC MANAGER REVIEW AND ROUTE RELEASE GUIDANCE.";
        return ProductDtos.CoordinationDraftSummary.builder()
                .id(UUID.nameUUIDFromBytes((missionId + ":" + subject).getBytes(java.nio.charset.StandardCharsets.UTF_8)).toString())
                .missionId(missionId)
                .reservationId(safe.getReservationId())
                .subject(subject)
                .family("USNS")
                .direction("OUTBOUND")
                .rawText(body)
                .recommendedAction(verdict.getRecommendedAction())
                .recipients(defaultWeatherRecipients())
                .sourceRefs(distinct(refs))
                .build();
    }

    public ProductDtos.PilotBriefSummary pilotBrief(String missionId, String since) {
        ProductDtos.MissionSummary mission = requireMission(missionId);
        ProductDtos.MissionWeatherVerdictSummary verdict = missionWeatherVerdict(missionId);
        ProductDtos.RouteImpactSummary impact = routeImpact(missionId, null);
        ProductDtos.CoordinationDraftSummary draft = coordinationDraft(missionId, null);
        ZonedDateTime sinceTime = blank(since) ? ZonedDateTime.now(ZoneOffset.UTC).minusHours(4) : parseTime(since);
        List<ProductDtos.WeatherSourceSummary> changes = new ArrayList<>();
        for (ProductDtos.WeatherSourceSummary source : verdict.getSources()) {
            if (source.getObservedAt() == null || !source.getObservedAt().isBefore(sinceTime)) {
                changes.add(source);
            }
        }
        String traceSummary = "Action " + impact.getAction() + " at " + Math.round(impact.getConfidence() * 100)
                + "% confidence; " + impact.getBlockingConstraintCount() + " blocking constraint(s); sources "
                + impact.getSourceRefs();
        List<String> sourceSummaryLines = pilotBriefSourceSummary(verdict, impact);
        String printable = "AIRSPACE PILOT BRIEF\n"
                + "MISSION: " + mission.getMissionNumber() + "\n"
                + "VERDICT: " + verdict.getAction() + " (" + Math.round(verdict.getConfidence() * 100) + "%)\n"
                + "RECOMMENDED: " + verdict.getRecommendedAction() + "\n"
                + "ROUTE IMPACT: " + impact.getRationale() + "\n"
                + "SOURCE DRIVERS:\n" + String.join("\n", sourceSummaryLines) + "\n"
                + "WHAT CHANGED: " + changes.size() + " weather/PIREP/NOTAM source(s)\n"
                + "TRACE: " + traceSummary + "\n";
        return ProductDtos.PilotBriefSummary.builder()
                .missionId(missionId)
                .missionNumber(mission.getMissionNumber())
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .verdict(verdict)
                .routeImpact(impact)
                .coordinationDraft(draft)
                .changes(changes)
                .sourceSummaryLines(sourceSummaryLines)
                .decisionTraceSummary(traceSummary)
                .printableText(printable)
                .build();
    }

    public List<ProductDtos.AffectedMissionSummary> affectedMissions(String sourceId, Integer limit) {
        int max = limit == null || limit <= 0 ? 50 : Math.min(limit, 250);
        ZonedDateTime now = ZonedDateTime.now(ZoneOffset.UTC);
        List<ProductDtos.AffectedMissionSummary> affected = new ArrayList<>();
        for (ProductDtos.MissionSummary mission : missions()) {
            ProductDtos.MissionWeatherVerdictSummary verdict = missionWeatherVerdict(mission.getId());
            ProductDtos.RouteImpactSummary impact = routeImpact(mission.getId(), null);
            List<String> refs = new ArrayList<>(impact.getSourceRefs());
            ZonedDateTime lastObserved = null;
            for (ProductDtos.WeatherSourceSummary source : verdict.getSources()) {
                refs.add(source.getFamily() + ":" + source.getId());
                refs.add(source.getId());
                if (source.getObservedAt() != null
                        && (lastObserved == null || source.getObservedAt().isAfter(lastObserved))) {
                    lastObserved = source.getObservedAt();
                }
            }
            List<String> distinctRefs = distinct(refs);
            if (!blank(sourceId) && distinctRefs.stream().noneMatch(ref -> ref.equals(sourceId) || ref.endsWith(":" + sourceId))) {
                continue;
            }
            boolean actionable = !"CLEAR".equals(verdict.getAction())
                    || verdict.getSourceCount() > 0
                    || impact.getImpactedSegmentCount() > 0
                    || impact.getBlockingConstraintCount() > 0;
            if (!actionable) {
                continue;
            }
            long ageSeconds = lastObserved == null ? -1 : Math.max(0, Duration.between(lastObserved, now).getSeconds());
            affected.add(ProductDtos.AffectedMissionSummary.builder()
                    .missionId(mission.getId())
                    .missionNumber(mission.getMissionNumber())
                    .status(mission.getStatus())
                    .action(verdict.getAction())
                    .priority(verdict.getPriority())
                    .confidence(Math.min(verdict.getConfidence(), impact.getConfidence() == 0.0 ? verdict.getConfidence() : impact.getConfidence()))
                    .sourceCount(verdict.getSourceCount())
                    .impactedSegmentCount(impact.getImpactedSegmentCount())
                    .blockingConstraintCount(impact.getBlockingConstraintCount())
                    .route("/missions/" + mission.getId())
                    .rationale(impact.getRationale())
                    .lastObservedAt(lastObserved)
                    .ageSeconds(ageSeconds)
                    .stale(lastObserved != null && lastObserved.plus(Duration.ofMinutes(90)).isBefore(now))
                    .guidanceLatencySeconds(ageSeconds)
                    .guidanceTargetMet(ageSeconds >= 0 && ageSeconds <= 5)
                    .sourceRefs(distinctRefs)
                    .build());
        }
        affected.sort((left, right) -> {
            int byPriority = severityRank(right.getPriority()) - severityRank(left.getPriority());
            if (byPriority != 0) {
                return byPriority;
            }
            int byAction = actionRank(right.getAction()) - actionRank(left.getAction());
            if (byAction != 0) {
                return byAction;
            }
            return String.valueOf(right.getLastObservedAt()).compareTo(String.valueOf(left.getLastObservedAt()));
        });
        return affected.size() > max ? new ArrayList<>(affected.subList(0, max)) : affected;
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
            return withGuidanceMetrics(new LinkedHashMap<>(persistenceService.metrics()));
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
        return withGuidanceMetrics(values);
    }

    private Map<String, Double> withGuidanceMetrics(Map<String, Double> values) {
        List<ProductDtos.AffectedMissionSummary> affected = affectedMissions(null, 250);
        long targetMet = affected.stream().filter(ProductDtos.AffectedMissionSummary::isGuidanceTargetMet).count();
        long stale = affected.stream().filter(ProductDtos.AffectedMissionSummary::isStale).count();
        double averageLatency = affected.stream()
                .filter(item -> item.getGuidanceLatencySeconds() >= 0)
                .mapToLong(ProductDtos.AffectedMissionSummary::getGuidanceLatencySeconds)
                .average()
                .orElse(0.0);
        values.put("product.weather.affectedMissions", (double) affected.size());
        values.put("product.weather.guidanceTargetMet", (double) targetMet);
        values.put("product.weather.guidanceTargetMissed", (double) (affected.size() - targetMet));
        values.put("product.weather.staleAffectedMissions", (double) stale);
        values.put("product.weather.averageGuidanceLatencySeconds", averageLatency);
        values.put("product.weather.guidanceTargetRate", affected.isEmpty() ? 1.0 : targetMet / (double) affected.size());
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

    private List<ProductDtos.MessageSummary> weatherMessagesForMission(String missionId) {
        List<ProductDtos.MessageSummary> out = new ArrayList<>();
        for (ProductDtos.MessageSummary message : messages()) {
            if ((message.getMissionId() == null || missionId.equals(message.getMissionId()))
                    && isWeatherFamily(message.getFamily(), message.getRawText(), message.getSubject())) {
                out.add(message);
            }
        }
        return out;
    }

    private List<ProductDtos.MessageSummary> guidanceMessagesForMission(String missionId) {
        List<ProductDtos.MessageSummary> out = new ArrayList<>();
        for (ProductDtos.MessageSummary message : messages()) {
            if ((message.getMissionId() == null || missionId.equals(message.getMissionId()))
                    && (isWeatherFamily(message.getFamily(), message.getRawText(), message.getSubject())
                    || isNotamTraffic(message.getFamily(), message.getRawText(), message.getSubject()))) {
                out.add(message);
            }
        }
        return out;
    }

    private boolean isWeatherFamily(String family, String rawText, String subject) {
        String text = (String.valueOf(family) + " " + String.valueOf(subject) + " " + String.valueOf(rawText))
                .toUpperCase(Locale.US);
        return text.contains("PIREP") || text.contains("SIGMET") || text.contains("AIRMET")
                || text.contains("METAR") || text.contains("TAF") || text.contains("CWAP")
                || text.contains("CWAF") || text.contains("CWA") || text.contains("/TB")
                || text.contains("/IC");
    }

    private boolean isNotamTraffic(String family, String rawText, String subject) {
        String text = (String.valueOf(family) + " " + String.valueOf(subject) + " " + String.valueOf(rawText))
                .toUpperCase(Locale.US);
        return text.contains("NOTAM") || text.contains("!FDC") || text.contains("!DCA")
                || text.contains("SNOWTAM") || text.contains("BIRDTAM") || text.contains("ASHTAM")
                || text.contains("GENOT") || "DOM".equals(String.valueOf(family).toUpperCase(Locale.US))
                || "FDC".equals(String.valueOf(family).toUpperCase(Locale.US));
    }

    private ProductDtos.WeatherSourceSummary sourceSummary(ProductDtos.MessageSummary message,
                                                           String severity,
                                                           String rationale) {
        ZonedDateTime now = ZonedDateTime.now(ZoneOffset.UTC);
        ZonedDateTime observedAt = message.getCreatedAt();
        long ageMinutes = ageMinutes(observedAt, now);
        boolean stale = observedAt != null
                && observedAt.plus(Duration.ofMinutes(90)).isBefore(now);
        return ProductDtos.WeatherSourceSummary.builder()
                .id(message.getId())
                .family(message.getFamily())
                .label(value(message.getSubject(), snippet(message.getRawText())))
                .route("/messages/" + message.getId())
                .severity(severity)
                .rationale(rationale)
                .observedAt(observedAt)
                .ageMinutes(ageMinutes)
                .agingCategory(agingCategory(ageMinutes, 90))
                .relevanceScore(weatherSourceRelevanceScore(ageMinutes, stale))
                .stale(stale)
                .build();
    }

    private String guidanceAction(ProductDtos.MessageSummary message) {
        String family = value(message.getFamily(), "").toUpperCase(Locale.US);
        String text = (String.valueOf(message.getSubject()) + " " + String.valueOf(message.getRawText())).toUpperCase(Locale.US);
        if (isNotamTraffic(family, message.getRawText(), message.getSubject())) {
            return text.contains("CLSD") || text.contains("CLOSED") || text.contains("PROHIBIT")
                    || text.contains("RESTRICT") || text.contains("TFR") ? "SEVERE" : "MODERATE";
        }
        if (family.contains("SIGMET") || text.contains("CONV") || text.contains("EMBD TS")) return "SEVERE";
        if (family.contains("PIREP") && (text.contains("SEV") || text.contains("URGENT"))) return "SEVERE";
        if (family.contains("PIREP") || text.contains("/TB") || text.contains("/IC")) return "OBSERVED";
        if (family.contains("AIRMET") || text.contains("TURB") || text.contains("ICE")) return "MODERATE";
        return "ADVISORY";
    }

    private String guidanceRationale(ProductDtos.MessageSummary message) {
        String text = (String.valueOf(message.getFamily()) + " " + String.valueOf(message.getRawText())).toUpperCase(Locale.US);
        if (isNotamTraffic(message.getFamily(), message.getRawText(), message.getSubject())) {
            return "NOTAM constraint is retained separately from CARF/ALTRV reservations and must be reviewed against route, altitude, and timing before release.";
        }
        if (text.contains("SIGMET") || text.contains("CONV")) {
            return "Convective weather can close route corridors and reduce sector capacity.";
        }
        if (text.contains("/IC") || text.contains("ICE")) {
            return "Icing report or forecast requires altitude and route-risk review.";
        }
        if (text.contains("/TB") || text.contains("TURB")) {
            return "Turbulence report or forecast requires crew briefing and altitude review.";
        }
        if (text.contains("METAR") || text.contains("TAF")) {
            return "Ceiling, visibility, or terminal forecast may affect release timing.";
        }
        return "Weather source retained for operational fusion and replay.";
    }

    private String strongestAction(ProductDtos.RouteImpactSummary impact, List<ProductDtos.WeatherSourceSummary> sources) {
        if ("BLOCKED".equals(impact.getAction())) return "BLOCKED";
        if ("REROUTE".equals(impact.getAction()) || "AVOID".equals(impact.getAction())) return impact.getAction();
        if (sources.stream().anyMatch(source -> "SEVERE".equals(source.getSeverity()))) return "AVOID";
        if (sources.stream().anyMatch(source -> "MODERATE".equals(source.getSeverity()) || "OBSERVED".equals(source.getSeverity()))) return "CAUTION";
        return sources.isEmpty() ? "CLEAR" : "MONITOR";
    }

    private String priorityFor(String action, List<ProductDtos.WeatherSourceSummary> sources) {
        if ("BLOCKED".equals(action) || "AVOID".equals(action) || "REROUTE".equals(action)) return "HIGH";
        if ("CAUTION".equals(action) || sources.stream().anyMatch(ProductDtos.WeatherSourceSummary::isStale)) return "MEDIUM";
        return "LOW";
    }

    private String recommendedActionFor(String action) {
        switch (action) {
            case "BLOCKED":
                return "Hold route release until blocking constraints clear or alternate route is accepted.";
            case "REROUTE":
            case "AVOID":
                return "Coordinate reroute or delay with weather desk, traffic manager, and mission owner.";
            case "CAUTION":
                return "Brief crew and review altitude/route exposure before release.";
            case "MONITOR":
                return "Monitor next weather update and reassess if hazard moves toward route.";
            default:
                return "Continue normal monitoring.";
        }
    }

    private String routeImpactRationale(OperationalDecisionResult decision, int notamConstraints) {
        String base = value(decision.getRationale(), "No route impact decision has been generated.");
        if (notamConstraints <= 0) {
            return base;
        }
        return base + " " + notamConstraints
                + " NOTAM constraint source(s) are retained separately from CARF/ALTRV reservations and require operational review.";
    }

    private List<String> pilotBriefSourceSummary(ProductDtos.MissionWeatherVerdictSummary verdict,
                                                 ProductDtos.RouteImpactSummary impact) {
        Map<String, Integer> counts = new LinkedHashMap<>();
        for (ProductDtos.WeatherSourceSummary source : verdict.getSources()) {
            String family = sourceRefFamily(source.getFamily() + ":" + source.getId());
            counts.put(family, counts.getOrDefault(family, 0) + 1);
        }
        for (String ref : impact.getSourceRefs()) {
            String family = sourceRefFamily(ref);
            counts.put(family, counts.getOrDefault(family, 0) + 1);
        }
        if (counts.isEmpty()) {
            return Collections.singletonList("- SOURCE: no explicit source refs retained");
        }
        List<String> lines = new ArrayList<>();
        for (Map.Entry<String, Integer> entry : counts.entrySet()) {
            lines.add("- " + entry.getKey() + ": " + entry.getValue() + " source reference(s)");
        }
        return lines;
    }

    private String sourceRefFamily(String ref) {
        String text = String.valueOf(ref).toUpperCase(Locale.US);
        if (text.startsWith("FDC:") || text.startsWith("DOM:") || text.contains("NOTAM")
                || text.contains("SNOWTAM") || text.contains("BIRDTAM")
                || text.contains("ASHTAM") || text.contains("GENOT")) {
            return "NOTAM";
        }
        if (text.startsWith("PIREP:") || text.contains("PIREP")) {
            return "PIREP";
        }
        if (text.startsWith("SIGMET:") || text.startsWith("AIRMET:") || text.startsWith("METAR:")
                || text.startsWith("TAF:") || text.startsWith("WEATHER:") || text.contains("WX")) {
            return "WEATHER";
        }
        if (text.startsWith("CARF:") || text.startsWith("ALTRV:") || text.startsWith("RESERVATION:")) {
            return "CARF/ALTRV";
        }
        if (text.startsWith("USNS:") || text.startsWith("MESSAGE:")) {
            return "USNS";
        }
        return "SOURCE";
    }

    private int severityRank(String severity) {
        if ("HIGH".equals(severity)) return 4;
        if ("MEDIUM".equals(severity)) return 3;
        if ("LOW".equals(severity)) return 2;
        if ("SEVERE".equals(severity)) return 4;
        if ("MODERATE".equals(severity) || "OBSERVED".equals(severity)) return 3;
        if ("ADVISORY".equals(severity)) return 2;
        return 1;
    }

    private int actionRank(String action) {
        if ("BLOCKED".equals(action)) return 7;
        if ("REROUTE".equals(action)) return 6;
        if ("AVOID".equals(action)) return 5;
        if ("CAUTION".equals(action)) return 4;
        if ("MONITOR".equals(action)) return 3;
        if ("CLEAR".equals(action)) return 2;
        return 1;
    }

    private long ageMinutes(ZonedDateTime observedAt, ZonedDateTime now) {
        if (observedAt == null) {
            return -1;
        }
        return Math.max(0, Duration.between(observedAt, now).toMinutes());
    }

    private String agingCategory(long ageMinutes, int recencyMinutes) {
        if (ageMinutes < 0) {
            return "UNKNOWN";
        }
        if (ageMinutes > recencyMinutes) {
            return "STALE";
        }
        if (ageMinutes > Math.max(15, recencyMinutes / 2)) {
            return "AGING";
        }
        return "CURRENT";
    }

    private double weatherSourceRelevanceScore(long ageMinutes, boolean stale) {
        if (ageMinutes < 0) {
            return 0.7;
        }
        if (stale) {
            return Math.max(0.15, 0.45 - Math.min(ageMinutes, 240) / 800.0);
        }
        return Math.max(0.45, 1.0 - ageMinutes / 240.0);
    }

    private double pirepRelevanceScore(boolean altitudeMatch,
                                      boolean timeMatch,
                                      boolean routeMatch,
                                      long ageMinutes,
                                      int recencyMinutes) {
        double ageFactor;
        if (ageMinutes < 0) {
            ageFactor = 0.7;
        } else {
            ageFactor = Math.max(0.15, 1.0 - ageMinutes / Math.max(1.0, recencyMinutes * 2.0));
        }
        double score = ageFactor;
        if (!altitudeMatch) {
            score *= 0.45;
        }
        if (!routeMatch) {
            score *= 0.45;
        }
        if (!timeMatch) {
            score *= 0.5;
        }
        return Math.max(0.0, Math.min(1.0, score));
    }

    private ParsedWeatherInputs parsedWeatherInputs(List<ProductDtos.MessageSummary> messages) {
        ParsedWeatherInputs inputs = new ParsedWeatherInputs();
        for (ProductDtos.MessageSummary message : messages) {
            String raw = value(message.getRawText(), message.getSubject());
            if (blank(raw)) {
                continue;
            }
            try {
                WeatherProductParseResult parsed = weatherProductParser.parse(raw, null);
                if (parsed.getProduct() != null) {
                    inputs.products.add(parsed.getProduct());
                }
                if (parsed.getPirepReport() != null) {
                    inputs.pireps.add(parsed.getPirepReport());
                }
                inputs.diagnostics.addAll(parsed.getWarnings());
                inputs.diagnostics.addAll(parsed.getErrors());
            } catch (RuntimeException ex) {
                inputs.diagnostics.add("Weather parse failed for " + message.getId() + ": " + ex.getMessage());
            }
        }
        return inputs;
    }

    private List<GeoCoordinate> routeForMission(ProductDtos.MissionDetail detail, String reservationId) {
        for (ProductDtos.ReservationSummary reservation : detail.getReservations()) {
            if (reservationId == null || reservationId.equals(reservation.getId())) {
                List<GeoCoordinate> route = routeFromText(reservation.getRawText());
                if (route.size() >= 2) {
                    return route;
                }
            }
        }
        return List.of(
                GeoCoordinate.builder().latitude(30.0).longitude(-150.5).altitude(24000.0).build(),
                GeoCoordinate.builder().latitude(30.0).longitude(-148.5).altitude(24000.0).build());
    }

    private List<GeoCoordinate> routeFromText(String raw) {
        if (raw == null) return Collections.emptyList();
        List<GeoCoordinate> points = new ArrayList<>();
        Matcher matcher = Pattern.compile("(\\d{2})(\\d{2})?\\s*N\\s*(\\d{3})(\\d{2})?\\s*W").matcher(raw.toUpperCase(Locale.US));
        while (matcher.find()) {
            double lat = Double.parseDouble(matcher.group(1)) + minutes(matcher.group(2)) / 60.0;
            double lon = -(Double.parseDouble(matcher.group(3)) + minutes(matcher.group(4)) / 60.0);
            points.add(GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(24000.0).build());
        }
        return points;
    }

    private double minutes(String value) {
        return value == null || value.isEmpty() ? 0.0 : Double.parseDouble(value);
    }

    private double distanceToRoute(GeoCoordinate point, List<GeoCoordinate> route) {
        if (point == null || route == null || route.isEmpty()) return Double.POSITIVE_INFINITY;
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i + 1 < route.size(); i++) {
            best = Math.min(best, Math.abs(point.crossTrackDistanceToPath(route.get(i), route.get(i + 1))));
            best = Math.min(best, point.distanceTo(route.get(i)));
            best = Math.min(best, point.distanceTo(route.get(i + 1)));
        }
        return best;
    }

    private List<String> defaultWeatherRecipients() {
        List<String> recipients = new ArrayList<>();
        recipients.add("FAA-ATCSCC");
        recipients.add("WEATHER-DESK");
        recipients.add("MISSION-OWNER");
        return recipients;
    }

    private List<String> distinct(List<String> values) {
        List<String> out = new ArrayList<>();
        for (String value : values) {
            if (!blank(value) && !out.contains(value)) {
                out.add(value);
            }
        }
        return out;
    }

    private static class ParsedWeatherInputs {
        private final List<WeatherProduct> products = new ArrayList<>();
        private final List<PirepReport> pireps = new ArrayList<>();
        private final List<String> diagnostics = new ArrayList<>();
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
