package org.tash.extensions.product.application;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.transaction.Transactional;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.OperationalDecisionAuditEnvelope;
import org.tash.extensions.engine.OperationalDecisionReplayBundle;
import org.tash.extensions.engine.OperationalDecisionRequest;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.feed.OperationalFeedIngestResult;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.product.persistence.entity.FeedArtifactEntity;
import org.tash.extensions.product.persistence.entity.HistoryEventEntity;
import org.tash.extensions.product.persistence.entity.MessageEntity;
import org.tash.extensions.product.persistence.entity.MissionEntity;
import org.tash.extensions.product.persistence.entity.NotamEntity;
import org.tash.extensions.product.persistence.entity.OperationalDecisionEntity;
import org.tash.extensions.product.persistence.entity.ReferencePointEntity;
import org.tash.extensions.product.persistence.entity.ReservationEntity;
import org.tash.extensions.product.persistence.entity.ApreqEntity;
import org.tash.extensions.product.persistence.entity.ApprovalEntity;
import org.tash.extensions.product.persistence.mapper.ProductEntityMapper;
import org.tash.extensions.product.persistence.repository.FeedArtifactJpaRepository;
import org.tash.extensions.product.persistence.repository.HistoryEventJpaRepository;
import org.tash.extensions.product.persistence.repository.MessageJpaRepository;
import org.tash.extensions.product.persistence.repository.MissionJpaRepository;
import org.tash.extensions.product.persistence.repository.NotamJpaRepository;
import org.tash.extensions.product.persistence.repository.OperationalDecisionJpaRepository;
import org.tash.extensions.product.persistence.repository.ReferencePointJpaRepository;
import org.tash.extensions.product.persistence.repository.WeatherJpaRepository;
import org.tash.extensions.product.persistence.repository.ApreqJpaRepository;
import org.tash.extensions.product.persistence.repository.ApprovalJpaRepository;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;
import org.tash.extensions.weather.avoid.PolygonalWeatherCell;
import org.tash.extensions.weather.product.WeatherConfidence;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductSource;
import org.tash.extensions.weather.product.WeatherProductType;
import org.tash.extensions.weather.product.WeatherValidityWindow;
import org.tash.extensions.workflow.ReservationWorkflowRecord;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.UUID;
import java.util.stream.Collectors;

@ApplicationScoped
public class ProductPersistenceService {
    private static final ObjectMapper JSON = new ObjectMapper().registerModule(new JavaTimeModule());
    private static final TypeReference<java.util.Map<String, Object>> MAP_TYPE = new TypeReference<java.util.Map<String, Object>>() {
    };

    private final MissionJpaRepository missionRepository;
    private final org.tash.extensions.product.persistence.repository.ReservationJpaRepository reservationRepository;
    private final MessageJpaRepository messageRepository;
    private final WeatherJpaRepository weatherRepository;
    private final OperationalDecisionJpaRepository decisionRepository;
    private final FeedArtifactJpaRepository feedArtifactRepository;
    private final HistoryEventJpaRepository historyRepository;
    private final ReferencePointJpaRepository referencePointRepository;
    private final NotamJpaRepository notamRepository;
    private final ApreqJpaRepository apreqRepository;
    private final ApprovalJpaRepository approvalRepository;
    private final ProductEntityMapper mapper = new ProductEntityMapper();

    public ProductPersistenceService(MissionJpaRepository missionRepository,
                                     org.tash.extensions.product.persistence.repository.ReservationJpaRepository reservationRepository,
                                     MessageJpaRepository messageRepository,
                                     WeatherJpaRepository weatherRepository,
                                     OperationalDecisionJpaRepository decisionRepository,
                                     FeedArtifactJpaRepository feedArtifactRepository,
                                     HistoryEventJpaRepository historyRepository,
                                     ReferencePointJpaRepository referencePointRepository,
                                     NotamJpaRepository notamRepository,
                                     ApreqJpaRepository apreqRepository,
                                     ApprovalJpaRepository approvalRepository) {
        this.missionRepository = missionRepository;
        this.reservationRepository = reservationRepository;
        this.messageRepository = messageRepository;
        this.weatherRepository = weatherRepository;
        this.decisionRepository = decisionRepository;
        this.feedArtifactRepository = feedArtifactRepository;
        this.historyRepository = historyRepository;
        this.referencePointRepository = referencePointRepository;
        this.notamRepository = notamRepository;
        this.apreqRepository = apreqRepository;
        this.approvalRepository = approvalRepository;
    }

    @Transactional
    public ProductDtos.MissionSummary createMission(ProductDtos.MissionRequest request, String id) {
        MissionEntity entity = new MissionEntity();
        entity.setId(UUID.fromString(id));
        entity.setMissionNumber(request.getMissionNumber());
        entity.setTitle(request.getTitle());
        entity.setStatus("DRAFT");
        entity.setRawText(request.getRawText());
        entity.setCreatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        entity.setUpdatedAt(entity.getCreatedAt());
        MissionEntity saved = missionRepository.save(entity);
        return missionSummary(saved);
    }

    @Transactional
    public ProductDtos.MissionSummary saveMission(ProductDtos.MissionSummary summary) {
        MissionEntity entity = missionRepository.findById(UUID.fromString(summary.getId()))
                .orElseGet(MissionEntity::new);
        entity.setId(UUID.fromString(summary.getId()));
        entity.setMissionNumber(summary.getMissionNumber());
        entity.setTitle(summary.getTitle());
        entity.setStatus(summary.getStatus());
        entity.setLockedBy(summary.getLockedBy());
        entity.setLockedAt(summary.getLockedAt());
        entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        return missionSummary(missionRepository.save(entity));
    }

    public List<ProductDtos.MissionSummary> missions() {
        return missionRepository.listAll().stream()
                .map(this::missionSummary)
                .collect(Collectors.toList());
    }

    public ProductDtos.MissionDetail mission(String id) {
        UUID missionId = UUID.fromString(id);
        MissionEntity mission = missionRepository.findById(missionId)
                .orElseThrow(() -> new IllegalArgumentException("Unknown mission: " + id));
        List<ProductDtos.ReservationSummary> reservations = reservationRepository.findByMissionId(missionId).stream()
                .map(this::reservationSummary)
                .collect(Collectors.toList());
        List<ProductDtos.MessageSummary> messages = messageRepository.findByMissionId(missionId).stream()
                .map(this::messageSummary)
                .collect(Collectors.toList());
        List<ProductDtos.HistoryEventSummary> history = historyRepository.findByAggregate("mission", id).stream()
                .map(this::historySummary)
                .collect(Collectors.toList());
        return ProductDtos.MissionDetail.builder()
                .mission(missionSummary(mission))
                .reservations(reservations)
                .messages(messages)
                .history(history)
                .build();
    }

    @Transactional
    public ProductDtos.ReservationSummary saveReservation(String missionId, ReservationWorkflowRecord record) {
        ReservationEntity entity = reservationRepository.findById(UUID.fromString(record.getId()))
                .orElseGet(ReservationEntity::new);
        entity.setId(UUID.fromString(record.getId()));
        entity.setMission(missionRepository.findById(UUID.fromString(missionId))
                .orElseThrow(() -> new IllegalArgumentException("Unknown mission: " + missionId)));
        entity.setReservationKey(record.getDraft() == null ? record.getId() : record.getDraft().getId());
        entity.setState(record.getState().name());
        entity.setRawText(record.getDraft() == null ? null : record.getDraft().getRawText());
        entity.setLockedBy(record.getLockOwner());
        entity.setLockedAt(record.getLockedAt());
        entity.setLastAnalysisJson(record.getLastAnalysis() == null ? null : org.tash.extensions.engine.CanonicalJson.write(record.getLastAnalysis()));
        entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        return reservationSummary(reservationRepository.save(entity));
    }

    @Transactional
    public ProductDtos.ReservationSummary saveReservation(ReservationWorkflowRecord record) {
        ReservationEntity entity = reservationRepository.findById(UUID.fromString(record.getId()))
                .orElseThrow(() -> new IllegalArgumentException("Unknown persisted reservation: " + record.getId()));
        entity.setReservationKey(record.getDraft() == null ? record.getId() : record.getDraft().getId());
        entity.setState(record.getState().name());
        entity.setRawText(record.getDraft() == null ? null : record.getDraft().getRawText());
        entity.setLockedBy(record.getLockOwner());
        entity.setLockedAt(record.getLockedAt());
        entity.setLastAnalysisJson(record.getLastAnalysis() == null ? null : org.tash.extensions.engine.CanonicalJson.write(record.getLastAnalysis()));
        entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        return reservationSummary(reservationRepository.save(entity));
    }

    @Transactional
    public ProductDtos.MessageSummary saveMessage(ProductDtos.MessageSummary summary) {
        MessageEntity entity = new MessageEntity();
        entity.setId(UUID.fromString(summary.getId()));
        entity.setMissionId(parseUuid(summary.getMissionId()));
        entity.setReservationId(parseUuid(summary.getReservationId()));
        entity.setFamily(summary.getFamily());
        entity.setDirection(summary.getDirection());
        entity.setStatus(summary.getStatus());
        entity.setSubject(summary.getSubject());
        entity.setRawText(summary.getRawText());
        entity.setCreatedAt(summary.getCreatedAt() == null ? ZonedDateTime.now(ZoneOffset.UTC) : summary.getCreatedAt());
        entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        return messageSummary(messageRepository.save(entity));
    }

    public List<ProductDtos.MessageSummary> messages() {
        return messageRepository.listAll().stream()
                .map(this::messageSummary)
                .collect(Collectors.toList());
    }

    public ProductDtos.MessageSummary message(String id) {
        return messageSummary(messageRepository.findById(UUID.fromString(id))
                .orElseThrow(() -> new IllegalArgumentException("Unknown message: " + id)));
    }

    @Transactional
    public void saveWeatherProduct(WeatherProduct product) {
        weatherRepository.save(mapper.toEntity(product));
    }

    @Transactional
    public UUID saveDecision(OperationalDecisionResult result) {
        OperationalDecisionEntity entity = mapper.toEntity(result);
        return decisionRepository.save(entity).getId();
    }

    public ProductDtos.DecisionSummary decision(String id) {
        OperationalDecisionEntity entity = decisionRepository.findById(UUID.fromString(id))
                .orElseThrow(() -> new IllegalArgumentException("Unknown decision: " + id));
        return ProductDtos.DecisionSummary.builder()
                .id(entity.getId().toString())
                .action(entity.getAction())
                .recommendedAction(entity.getRecommendedAction())
                .confidence(entity.getConfidence())
                .rationale(entity.getRationale())
                .resultJson(entity.getResultJson())
                .auditJson(entity.getAuditJson())
                .replayJson(entity.getReplayJson())
                .build();
    }

    public OperationalDecisionResult decisionResult(String id) {
        OperationalDecisionEntity entity = decisionRepository.findById(UUID.fromString(id))
                .orElseThrow(() -> new IllegalArgumentException("Unknown decision: " + id));
        try {
            JsonNode root = JSON.readTree(entity.getResultJson());
            return OperationalDecisionResult.builder()
                    .action(enumValue(WeatherDecisionAction.class, text(root, "action")))
                    .recommendedAction(enumValue(WeatherRecommendedAction.class, text(root, "recommendedAction")))
                    .rationale(text(root, "rationale"))
                    .confidence(root.path("confidence").asDouble(entity.getConfidence()))
                    .weatherProducts(weatherProducts(root.path("weatherProducts")))
                    .auditEnvelope(decisionAuditEnvelope(id))
                    .replayBundle(decisionReplayBundle(id))
                    .build();
        } catch (Exception e) {
            throw new IllegalArgumentException("Unable to reload persisted decision result: " + id, e);
        }
    }

    public OperationalDecisionRequest decisionRequest(String id) {
        return decisionReplayBundle(id).getRequest();
    }

    public OperationalDecisionResult rehydrateDecision(String id) {
        return decisionResult(id);
    }

    public ProductDtos.ReservationSummary rehydrateReservation(String id) {
        return reservationSummary(reservationRepository.findById(UUID.fromString(id))
                .orElseThrow(() -> new IllegalArgumentException("Unknown reservation: " + id)));
    }

    public OperationalDecisionReplayBundle decisionReplayBundle(String id) {
        OperationalDecisionEntity entity = decisionRepository.findById(UUID.fromString(id))
                .orElseThrow(() -> new IllegalArgumentException("Unknown decision: " + id));
        try {
            JsonNode root = JSON.readTree(entity.getReplayJson());
            JsonNode request = root.path("request");
            return OperationalDecisionReplayBundle.builder()
                    .bundleVersion(text(root, "bundleVersion"))
                    .engineVersion(text(root, "engineVersion"))
                    .ruleCatalogVersion(text(root, "ruleCatalogVersion"))
                    .expectedAction(text(root, "expectedAction"))
                    .expectedRecommendedAction(text(root, "expectedRecommendedAction"))
                    .expectedResultHash(text(root, "expectedResultHash"))
                    .request(OperationalDecisionRequest.builder()
                            .decisionTime(zoned(text(request, "decisionTime")))
                            .rawUsnsMessages(strings(request.path("rawUsnsMessages")))
                            .rawCarfMessages(strings(request.path("rawCarfMessages")))
                            .route(route(request.path("route")))
                            .build())
                    .auditEnvelope(decisionAuditEnvelope(id))
                    .build();
        } catch (Exception e) {
            throw new IllegalArgumentException("Unable to reload persisted replay bundle: " + id, e);
        }
    }

    public OperationalDecisionAuditEnvelope decisionAuditEnvelope(String id) {
        OperationalDecisionEntity entity = decisionRepository.findById(UUID.fromString(id))
                .orElseThrow(() -> new IllegalArgumentException("Unknown decision: " + id));
        try {
            JsonNode root = JSON.readTree(entity.getAuditJson());
            return OperationalDecisionAuditEnvelope.builder()
                    .engineVersion(text(root, "engineVersion"))
                    .ruleCatalogVersion(text(root, "ruleCatalogVersion"))
                    .requestHash(text(root, "requestHash"))
                    .resultHash(text(root, "resultHash"))
                    .configHash(text(root, "configHash"))
                    .timestamp(zoned(text(root, "timestamp")))
                    .signingKeyId(text(root, "signingKeyId"))
                    .signature(text(root, "signature"))
                    .inputSourceHashes(strings(root.path("inputSourceHashes")))
                    .build();
        } catch (Exception e) {
            throw new IllegalArgumentException("Unable to reload persisted audit envelope: " + id, e);
        }
    }

    @Transactional
    public ProductDtos.FeedArtifactSummary saveFeedArtifact(OperationalFeedIngestResult result) {
        FeedArtifactEntity entity = new FeedArtifactEntity();
        entity.setId(UUID.fromString(result.getEnvelope().getId()));
        entity.setSourceId(result.getEnvelope().getSourceId());
        entity.setFeedType(result.getEnvelope().getType() == null ? "UNKNOWN" : result.getEnvelope().getType().name());
        entity.setPayloadHash(result.getRawPayloadHash());
        entity.setRawPayload(result.getEnvelope().getRawPayload());
        entity.setAccepted(result.isAccepted());
        entity.setReceivedAt(result.getEnvelope().getReceivedAt());
        java.util.Map<String, Object> diagnostics = new java.util.LinkedHashMap<>();
        diagnostics.put("warnings", result.getWarnings());
        diagnostics.put("errors", result.getErrors());
        diagnostics.put("downstreamArtifactIds", result.getDownstreamArtifactIds());
        entity.setDiagnosticsJson(org.tash.extensions.engine.CanonicalJson.write(diagnostics));
        return feedArtifactSummary(feedArtifactRepository.save(entity));
    }

    public List<ProductDtos.FeedArtifactSummary> feedArtifacts() {
        return feedArtifactRepository.listAll().stream()
                .map(this::feedArtifactSummary)
                .collect(Collectors.toList());
    }

    public ProductDtos.FeedArtifactSummary feedArtifact(String id) {
        return feedArtifactSummary(feedArtifactRepository.findById(UUID.fromString(id))
                .orElseThrow(() -> new IllegalArgumentException("Unknown feed artifact: " + id)));
    }

    @Transactional
    public ProductDtos.HistoryEventSummary saveHistory(ProductDtos.HistoryEventSummary event) {
        HistoryEventEntity entity = new HistoryEventEntity();
        entity.setId(UUID.fromString(event.getId()));
        entity.setAggregateType(event.getAggregateType());
        entity.setAggregateId(event.getAggregateId());
        entity.setEventType(event.getEventType());
        entity.setActor(event.getActor());
        entity.setNote(event.getNote());
        entity.setCreatedAt(event.getCreatedAt());
        entity.setEventJson(org.tash.extensions.engine.CanonicalJson.write(event));
        return historySummary(historyRepository.save(entity));
    }

    public List<ProductDtos.HistoryEventSummary> history() {
        return historyRepository.listAll().stream()
                .map(this::historySummary)
                .collect(Collectors.toList());
    }

    @Transactional
    public ProductDtos.ReferencePointSummary saveReferencePoint(ProductDtos.ReferencePointRequest request) {
        ReferencePointEntity entity = new ReferencePointEntity();
        entity.setIdentifier(request.getIdentifier());
        entity.setPointType(value(request.getPointType(), "FIX"));
        entity.setLatitude(request.getLatitude());
        entity.setLongitude(request.getLongitude());
        entity.setAltitudeFeet(request.getAltitudeFeet());
        entity.setSource(value(request.getSource(), "operator"));
        entity.setMetadataJson(request.getMetadataJson());
        entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        return referencePointSummary(referencePointRepository.save(entity));
    }

    public List<ProductDtos.ReferencePointSummary> referencePoints(String pointType) {
        List<ReferencePointEntity> values = pointType == null || pointType.trim().isEmpty()
                ? referencePointRepository.listAll()
                : referencePointRepository.findByType(pointType);
        return values.stream()
                .map(this::referencePointSummary)
                .collect(Collectors.toList());
    }

    @Transactional
    public ProductDtos.ReservationSupplementSummary saveSupplement(String reservationId,
                                                                   ProductDtos.ReservationSupplementRequest request) {
        String kind = value(request.getKind(), "COORDINATION").trim().toUpperCase(Locale.US);
        if ("NOTAM".equals(kind)) {
            NotamEntity entity = new NotamEntity();
            entity.setReservationId(UUID.fromString(reservationId));
            entity.setNotamType(value(request.getTitle(), "DOMESTIC"));
            entity.setRawText(request.getText());
            entity.setParsedJson(request.getStatus());
            entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
            return notamSummary(notamRepository.save(entity));
        }
        if ("APREQ".equals(kind)) {
            ApreqEntity entity = new ApreqEntity();
            entity.setReservationId(UUID.fromString(reservationId));
            entity.setStatus(value(request.getStatus(), "DRAFT"));
            entity.setRequestText(request.getText());
            entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
            return apreqSummary(apreqRepository.save(entity));
        }
        ApprovalEntity entity = new ApprovalEntity();
        entity.setReservationId(UUID.fromString(reservationId));
        entity.setApprovalType(kind);
        entity.setStatus(value(request.getStatus(), "OPEN"));
        entity.setActor(request.getActor());
        entity.setNote(request.getText());
        entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        return approvalSummary(entity, request.getTitle(), approvalRepository.save(entity));
    }

    public List<ProductDtos.ReservationSupplementSummary> supplements(String reservationId) {
        UUID id = UUID.fromString(reservationId);
        List<ProductDtos.ReservationSupplementSummary> values = new ArrayList<>();
        values.addAll(notamRepository.findByReservationId(id).stream().map(this::notamSummary).collect(Collectors.toList()));
        values.addAll(apreqRepository.findByReservationId(id).stream().map(this::apreqSummary).collect(Collectors.toList()));
        values.addAll(approvalRepository.findByReservationId(id).stream()
                .map(entity -> approvalSummary(entity, entity.getApprovalType(), entity))
                .collect(Collectors.toList()));
        return values;
    }

    @Transactional
    public ProductDtos.ReservationSupplementSummary transitionSupplement(String supplementId,
                                                                         ProductDtos.SupplementTransitionRequest request) {
        ProductDtos.SupplementTransitionRequest safe = request == null ? new ProductDtos.SupplementTransitionRequest() : request;
        String status = value(safe.getStatus(), "OPEN").trim().toUpperCase(Locale.US);
        UUID id = UUID.fromString(supplementId);
        java.util.Optional<NotamEntity> notam = notamRepository.findById(id);
        if (notam.isPresent()) {
            NotamEntity entity = notam.get();
            entity.setParsedJson(status);
            entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
            return notamSummary(notamRepository.save(entity));
        }
        java.util.Optional<ApreqEntity> apreq = apreqRepository.findById(id);
        if (apreq.isPresent()) {
            ApreqEntity entity = apreq.get();
            entity.setStatus(status);
            if (safe.getNote() != null) {
                entity.setResponseText(safe.getNote());
            }
            entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
            return apreqSummary(apreqRepository.save(entity));
        }
        java.util.Optional<ApprovalEntity> approval = approvalRepository.findById(id);
        if (approval.isPresent()) {
            ApprovalEntity entity = approval.get();
            entity.setStatus(status);
            entity.setActor(safe.getActor());
            if (safe.getNote() != null) {
                entity.setNote(safe.getNote());
            }
            entity.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
            return approvalSummary(entity, entity.getApprovalType(), approvalRepository.save(entity));
        }
        throw new IllegalArgumentException("Unknown reservation supplement: " + supplementId);
    }

    public List<ProductDtos.SearchResultSummary> searchMissions(String query) {
        return search(query).stream().filter(result -> "mission".equals(result.getType())).collect(Collectors.toList());
    }

    public List<ProductDtos.SearchResultSummary> searchMessages(String query) {
        return search(query).stream().filter(result -> "message".equals(result.getType())).collect(Collectors.toList());
    }

    public List<ProductDtos.SearchResultSummary> searchDecisions(String query) {
        String needle = normalize(query);
        List<ProductDtos.SearchResultSummary> results = new ArrayList<>();
        for (OperationalDecisionEntity decision : decisionRepository.listAll()) {
            if (matches(needle, decision.getAction(), decision.getRecommendedAction(), decision.getRationale(), decision.getResultJson())) {
                results.add(ProductDtos.SearchResultSummary.builder()
                        .id(decision.getId().toString())
                        .type("decision")
                        .title(value(decision.getAction(), "Decision"))
                        .status(decision.getRecommendedAction())
                        .snippet(snippet(decision.getRationale()))
                        .route("/decisions/" + decision.getId())
                        .updatedAt(decision.getUpdatedAt())
                        .build());
            }
        }
        return results;
    }

    public List<ProductDtos.SearchResultSummary> searchHistory(String query) {
        return search(query).stream().filter(result -> "history".equals(result.getType())).collect(Collectors.toList());
    }

    public List<ProductDtos.SearchResultSummary> search(String query) {
        String needle = normalize(query);
        List<ProductDtos.SearchResultSummary> results = new ArrayList<>();
        for (ProductDtos.MissionSummary mission : missions()) {
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
        for (ProductDtos.MessageSummary message : messages()) {
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
        for (ProductDtos.HistoryEventSummary event : history()) {
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

    public Map<String, Double> metrics() {
        Map<String, Double> values = new LinkedHashMap<>();
        values.put("product.missions", (double) missionRepository.count());
        values.put("product.reservations", (double) reservationRepository.count());
        values.put("product.messages", (double) messageRepository.count());
        values.put("product.feedArtifacts", (double) feedArtifactRepository.count());
        long acceptedFeeds = feedArtifactRepository.listAll().stream().filter(FeedArtifactEntity::isAccepted).count();
        values.put("product.feedAccepted", (double) acceptedFeeds);
        values.put("product.feedRejected", (double) feedArtifactRepository.count() - acceptedFeeds);
        values.put("product.decisions", (double) decisionRepository.count());
        values.put("product.historyEvents", (double) historyRepository.count());
        values.put("product.referencePoints", (double) referencePointRepository.count());
        values.put("product.weatherProducts", (double) weatherRepository.count());
        values.put("product.supplements", (double) (notamRepository.count() + apreqRepository.count() + approvalRepository.count()));
        values.put("product.notams", (double) notamRepository.count());
        values.put("product.apreqs", (double) apreqRepository.count());
        values.put("product.approvals", (double) approvalRepository.count());
        values.put("product.records", values.values().stream().mapToDouble(Double::doubleValue).sum());
        return values;
    }

    private ProductDtos.MissionSummary missionSummary(MissionEntity entity) {
        return ProductDtos.MissionSummary.builder()
                .id(entity.getId().toString())
                .missionNumber(entity.getMissionNumber())
                .title(entity.getTitle())
                .status(entity.getStatus())
                .lockedBy(entity.getLockedBy())
                .lockedAt(entity.getLockedAt())
                .reservationCount(entity.getReservations() == null ? 0 : entity.getReservations().size())
                .updatedAt(entity.getUpdatedAt())
                .build();
    }

    private ProductDtos.MessageSummary messageSummary(MessageEntity entity) {
        return ProductDtos.MessageSummary.builder()
                .id(entity.getId().toString())
                .missionId(entity.getMissionId() == null ? null : entity.getMissionId().toString())
                .reservationId(entity.getReservationId() == null ? null : entity.getReservationId().toString())
                .family(entity.getFamily())
                .direction(entity.getDirection())
                .status(entity.getStatus())
                .subject(entity.getSubject())
                .rawText(entity.getRawText())
                .createdAt(entity.getCreatedAt())
                .build();
    }

    private ProductDtos.ReservationSummary reservationSummary(ReservationEntity entity) {
        java.util.List<String> diagnostics = new java.util.ArrayList<>();
        if (entity.getLastAnalysisJson() != null && entity.getLastAnalysisJson().contains("diagnostics")) {
            diagnostics.add("analysis persisted");
        }
        return ProductDtos.ReservationSummary.builder()
                .id(entity.getId().toString())
                .missionId(entity.getMission() == null ? null : entity.getMission().getId().toString())
                .state(org.tash.extensions.workflow.ReservationWorkflowState.valueOf(entity.getState()))
                .rawText(entity.getRawText())
                .lockedBy(entity.getLockedBy())
                .lockedAt(entity.getLockedAt())
                .conflictCount(0)
                .diagnostics(diagnostics)
                .build();
    }

    private ProductDtos.FeedArtifactSummary feedArtifactSummary(FeedArtifactEntity entity) {
        java.util.List<String> diagnostics = new java.util.ArrayList<>();
        java.util.List<String> downstreamArtifactIds = new java.util.ArrayList<>();
        if (entity.getDiagnosticsJson() != null) {
            try {
                java.util.Map<String, Object> diagnosticMap = JSON.readValue(entity.getDiagnosticsJson(), MAP_TYPE);
                appendStrings(diagnostics, diagnosticMap.get("warnings"));
                appendStrings(diagnostics, diagnosticMap.get("errors"));
                appendStrings(downstreamArtifactIds, diagnosticMap.get("downstreamArtifactIds"));
            } catch (Exception ignored) {
                diagnostics.add(entity.getDiagnosticsJson());
            }
        }
        return ProductDtos.FeedArtifactSummary.builder()
                .id(entity.getId().toString())
                .sourceId(entity.getSourceId())
                .type(entity.getFeedType())
                .accepted(entity.isAccepted())
                .rawPayloadHash(entity.getPayloadHash())
                .rawPayload(entity.getRawPayload())
                .receivedAt(entity.getReceivedAt())
                .downstreamArtifactIds(downstreamArtifactIds)
                .diagnostics(diagnostics)
                .build();
    }

    private void appendStrings(java.util.List<String> target, Object value) {
        if (value instanceof Iterable<?>) {
            for (Object item : (Iterable<?>) value) {
                if (item != null) {
                    target.add(String.valueOf(item));
                }
            }
        } else if (value != null) {
            target.add(String.valueOf(value));
        }
    }

    private ProductDtos.HistoryEventSummary historySummary(HistoryEventEntity entity) {
        return ProductDtos.HistoryEventSummary.builder()
                .id(entity.getId().toString())
                .aggregateType(entity.getAggregateType())
                .aggregateId(entity.getAggregateId())
                .eventType(entity.getEventType())
                .actor(entity.getActor())
                .note(entity.getNote())
                .createdAt(entity.getCreatedAt())
                .build();
    }

    private ProductDtos.ReferencePointSummary referencePointSummary(ReferencePointEntity entity) {
        return ProductDtos.ReferencePointSummary.builder()
                .id(entity.getId().toString())
                .identifier(entity.getIdentifier())
                .pointType(entity.getPointType())
                .latitude(entity.getLatitude())
                .longitude(entity.getLongitude())
                .altitudeFeet(entity.getAltitudeFeet())
                .source(entity.getSource())
                .metadataJson(entity.getMetadataJson())
                .updatedAt(entity.getUpdatedAt())
                .build();
    }

    private ProductDtos.ReservationSupplementSummary notamSummary(NotamEntity entity) {
        return ProductDtos.ReservationSupplementSummary.builder()
                .id(entity.getId().toString())
                .reservationId(entity.getReservationId() == null ? null : entity.getReservationId().toString())
                .kind("NOTAM")
                .status(value(entity.getParsedJson(), "DRAFT"))
                .title(entity.getNotamType())
                .text(entity.getRawText())
                .updatedAt(entity.getUpdatedAt())
                .build();
    }

    private ProductDtos.ReservationSupplementSummary apreqSummary(ApreqEntity entity) {
        return ProductDtos.ReservationSupplementSummary.builder()
                .id(entity.getId().toString())
                .reservationId(entity.getReservationId() == null ? null : entity.getReservationId().toString())
                .kind("APREQ")
                .status(entity.getStatus())
                .title("APREQ")
                .text(entity.getRequestText())
                .updatedAt(entity.getUpdatedAt())
                .build();
    }

    private ProductDtos.ReservationSupplementSummary approvalSummary(ApprovalEntity source, String title, ApprovalEntity entity) {
        return ProductDtos.ReservationSupplementSummary.builder()
                .id(entity.getId().toString())
                .reservationId(entity.getReservationId() == null ? null : entity.getReservationId().toString())
                .kind(entity.getApprovalType())
                .status(entity.getStatus())
                .title(title)
                .text(entity.getNote())
                .actor(entity.getActor())
                .updatedAt(entity.getUpdatedAt())
                .build();
    }

    private UUID parseUuid(String value) {
        return value == null || value.trim().isEmpty() ? null : UUID.fromString(value);
    }

    private static String text(JsonNode node, String field) {
        JsonNode value = node == null ? null : node.get(field);
        return value == null || value.isNull() ? null : value.asText();
    }

    private static ZonedDateTime zoned(String value) {
        return value == null || value.trim().isEmpty() ? null : ZonedDateTime.parse(value);
    }

    private static <T extends Enum<T>> T enumValue(Class<T> type, String value) {
        return value == null || value.trim().isEmpty() ? null : Enum.valueOf(type, value);
    }

    private static List<String> strings(JsonNode array) {
        List<String> values = new ArrayList<>();
        if (array != null && array.isArray()) {
            for (JsonNode item : array) {
                values.add(item.asText());
            }
        }
        return values;
    }

    private static List<GeoCoordinate> route(JsonNode array) {
        List<GeoCoordinate> values = new ArrayList<>();
        if (array != null && array.isArray()) {
            for (JsonNode item : array) {
                if (item.has("latitude") && item.has("longitude")) {
                    values.add(GeoCoordinate.builder()
                            .latitude(item.path("latitude").asDouble())
                            .longitude(item.path("longitude").asDouble())
                            .altitude(item.path("altitude").asDouble(0.0))
                            .build());
                }
            }
        }
        return values;
    }

    private static List<WeatherProduct> weatherProducts(JsonNode array) {
        List<WeatherProduct> values = new ArrayList<>();
        if (array != null && array.isArray()) {
            for (JsonNode item : array) {
                List<GeoCoordinate> geometry = route(item.path("geometry"));
                ZonedDateTime start = zoned(text(item.path("validity"), "validStart"));
                ZonedDateTime end = zoned(text(item.path("validity"), "validEnd"));
                Double lower = nullableDouble(item, "lowerAltitudeFeet");
                Double upper = nullableDouble(item, "upperAltitudeFeet");
                PolygonalWeatherCell hazard = geometry.size() >= 3 ? PolygonalWeatherCell.builder()
                        .id(text(item, "id"))
                        .type(weatherElement(item))
                        .severity(HazardSeverity.MODERATE)
                        .startTime(start)
                        .endTime(end)
                        .minAltitude(lower == null ? 0.0 : lower)
                        .maxAltitude(upper == null ? 60000.0 : upper)
                        .vertices(geometry)
                        .build() : null;
                values.add(WeatherProduct.builder()
                        .id(text(item, "id"))
                        .type(enumValue(WeatherProductType.class, text(item, "type")))
                        .source(enumValue(WeatherProductSource.class, text(item, "source")))
                        .sourceProduct(text(item, "sourceProduct"))
                        .provider(text(item, "provider"))
                        .rawText(text(item, "rawText"))
                        .issuedAt(zoned(text(item, "issuedAt")))
                        .receivedAt(zoned(text(item, "receivedAt")))
                        .validity(start == null && end == null ? null : WeatherValidityWindow.builder()
                                .validStart(start)
                                .validEnd(end)
                                .build())
                        .forecastHour(item.hasNonNull("forecastHour") ? item.get("forecastHour").asInt() : null)
                        .confidence(item.path("confidence").hasNonNull("value") ? WeatherConfidence.builder()
                                .value(item.path("confidence").path("value").asDouble())
                                .basis(text(item.path("confidence"), "basis"))
                                .build() : null)
                        .provenance(text(item, "provenance"))
                        .geometry(geometry)
                        .hazard(hazard)
                        .lowerAltitudeFeet(lower)
                        .upperAltitudeFeet(upper)
                        .echoTopFeet(nullableDouble(item, "echoTopFeet"))
                        .growthTrend(nullableDouble(item, "growthTrend"))
                        .stormPhase(text(item, "stormPhase"))
                        .build());
            }
        }
        return values;
    }

    private static Double nullableDouble(JsonNode node, String field) {
        return node != null && node.hasNonNull(field) ? node.get(field).asDouble() : null;
    }

    private static WeatherElementType weatherElement(JsonNode product) {
        WeatherProductType type = enumValue(WeatherProductType.class, text(product, "type"));
        if (type == WeatherProductType.ICING) {
            return WeatherElementType.ICING;
        }
        if (type == WeatherProductType.TURBULENCE) {
            return WeatherElementType.TURBULENCE;
        }
        if (type == WeatherProductType.CEILING) {
            return WeatherElementType.CEILING;
        }
        if (type == WeatherProductType.VISIBILITY) {
            return WeatherElementType.VISIBILITY;
        }
        return WeatherElementType.CONVECTION;
    }

    private boolean matches(String needle, String... values) {
        if (needle == null || needle.trim().isEmpty()) {
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

    private String value(String value, String fallback) {
        return value == null || value.trim().isEmpty() ? fallback : value;
    }
}
