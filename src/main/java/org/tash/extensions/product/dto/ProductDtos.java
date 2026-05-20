package org.tash.extensions.product.dto;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.feed.OperationalFeedBatchResult;
import org.tash.extensions.workflow.ReservationWorkflowState;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public final class ProductDtos {
    private ProductDtos() {
    }

    @Data
    public static class MissionRequest {
        private String missionNumber;
        private String title;
        private String rawText;
        private String actor;
    }

    @Data
    @Builder(toBuilder = true)
    public static class MissionSummary {
        private String id;
        private String missionNumber;
        private String title;
        private String status;
        private String lockedBy;
        private ZonedDateTime lockedAt;
        private int reservationCount;
        private ZonedDateTime updatedAt;
    }

    @Data
    @Builder
    public static class MissionDetail {
        private MissionSummary mission;
        @Builder.Default
        private List<ReservationSummary> reservations = new ArrayList<>();
        @Builder.Default
        private List<MessageSummary> messages = new ArrayList<>();
        @Builder.Default
        private List<HistoryEventSummary> history = new ArrayList<>();
    }

    @Data
    public static class ReservationRequest {
        private String rawText;
        private String actor;
        private String note;
    }

    @Data
    @Builder
    public static class ReservationSummary {
        private String id;
        private String missionId;
        private ReservationWorkflowState state;
        private String rawText;
        private String lockedBy;
        private ZonedDateTime lockedAt;
        private int conflictCount;
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }

    @Data
    public static class MessageRequest {
        private String missionId;
        private String reservationId;
        private String family;
        private String direction;
        private String subject;
        private String rawText;
        private String actor;
    }

    @Data
    @Builder
    public static class MessageSummary {
        private String id;
        private String missionId;
        private String reservationId;
        private String family;
        private String direction;
        private String status;
        private String subject;
        private String rawText;
        private ZonedDateTime createdAt;
    }

    @Data
    public static class FeedIngestRequest {
        private String sourceId;
        private String type;
        private String rawPayload;
        private boolean required;
    }

    @Data
    @Builder
    public static class FeedArtifactSummary {
        private String id;
        private String sourceId;
        private String type;
        private boolean accepted;
        private String rawPayloadHash;
        private String rawPayload;
        private ZonedDateTime receivedAt;
        @Builder.Default
        private List<String> downstreamArtifactIds = new ArrayList<>();
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }

    @Data
    @Builder
    public static class FeedTransactionSummary {
        private String id;
        private String type;
        private String status;
        private boolean supported;
        private String normalizedText;
        @Builder.Default
        private List<String> warnings = new ArrayList<>();
        @Builder.Default
        private List<String> errors = new ArrayList<>();
    }

    @Data
    public static class DecisionEvaluateRequest {
        private List<String> rawUsnsMessages = new ArrayList<>();
        private List<String> rawCarfMessages = new ArrayList<>();
        private List<List<Double>> route = new ArrayList<>();
        private String decisionTime;
    }

    @Data
    @Builder
    public static class DecisionSummary {
        private String id;
        private String action;
        private String recommendedAction;
        private double confidence;
        private String rationale;
        private String resultJson;
        private String auditJson;
        private String replayJson;
        private OperationalDecisionResult result;
    }

    @Data
    @Builder
    public static class HistoryEventSummary {
        private String id;
        private String aggregateType;
        private String aggregateId;
        private String eventType;
        private String actor;
        private String note;
        private ZonedDateTime createdAt;
    }

    @Data
    @Builder
    public static class SearchResultSummary {
        private String id;
        private String type;
        private String title;
        private String status;
        private String snippet;
        private String route;
        private ZonedDateTime updatedAt;
    }

    @Data
    public static class ReferencePointRequest {
        private String identifier;
        private String pointType;
        private double latitude;
        private double longitude;
        private Double altitudeFeet;
        private String source;
        private String metadataJson;
    }

    @Data
    public static class ReferenceDataImportRequest {
        private String payload;
        private String actor;
        private boolean apply;
    }

    @Data
    @Builder
    public static class ReferenceDataImportResult {
        private boolean accepted;
        private int parsedCount;
        private int appliedCount;
        @Builder.Default
        private List<String> warnings = new ArrayList<>();
        @Builder.Default
        private List<String> errors = new ArrayList<>();
        @Builder.Default
        private List<ReferencePointSummary> points = new ArrayList<>();
    }

    @Data
    @Builder
    public static class ReferencePointSummary {
        private String id;
        private String identifier;
        private String pointType;
        private double latitude;
        private double longitude;
        private Double altitudeFeet;
        private String source;
        private String metadataJson;
        private ZonedDateTime updatedAt;
    }

    @Data
    public static class ReservationSupplementRequest {
        private String kind;
        private String status;
        private String title;
        private String text;
        private String actor;
    }

    @Data
    public static class SupplementTransitionRequest {
        private String status;
        private String actor;
        private String note;
    }

    @Data
    @Builder(toBuilder = true)
    public static class ReservationSupplementSummary {
        private String id;
        private String reservationId;
        private String kind;
        private String status;
        private String title;
        private String text;
        private String actor;
        private ZonedDateTime updatedAt;
    }

    @Data
    @Builder
    public static class RuntimeSummary {
        private OperationalFeedBatchResult feedBatch;
        private DecisionSummary decision;
        private Map<String, Double> metrics;
    }
}
