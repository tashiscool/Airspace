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
    public static class PirepRelevanceRequest {
        private List<List<Double>> route = new ArrayList<>();
        private String reservationId;
        private Double lowerAltitudeFeet;
        private Double upperAltitudeFeet;
        private Double altitudeToleranceFeet;
        private Integer recencyMinutes;
        private Double corridorNauticalMiles;
    }

    @Data
    public static class CoordinationDraftRequest {
        private String hazardOrDecisionId;
        private String missionId;
        private String reservationId;
        private String actor;
    }

    @Data
    @Builder
    public static class WeatherSourceSummary {
        private String id;
        private String family;
        private String label;
        private String route;
        private String severity;
        private String rationale;
        private ZonedDateTime observedAt;
        private long ageMinutes;
        private String agingCategory;
        private double relevanceScore;
        private boolean stale;
    }

    @Data
    @Builder
    public static class MissionWeatherVerdictSummary {
        private String missionId;
        private String action;
        private String priority;
        private double confidence;
        private int sourceCount;
        private boolean stale;
        private String summary;
        private String recommendedAction;
        @Builder.Default
        private List<WeatherSourceSummary> sources = new ArrayList<>();
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }

    @Data
    @Builder
    public static class RouteImpactSummary {
        private String missionId;
        private String reservationId;
        private String action;
        private String recommendedAction;
        private double confidence;
        private String rationale;
        private int impactedSegmentCount;
        private int blockingConstraintCount;
        @Builder.Default
        private List<String> impactedSegments = new ArrayList<>();
        @Builder.Default
        private List<String> sourceRefs = new ArrayList<>();
        @Builder.Default
        private List<String> avoidanceCandidates = new ArrayList<>();
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }

    @Data
    @Builder
    public static class PirepRelevanceResult {
        private String missionId;
        private int totalPireps;
        private int relevantCount;
        private int staleCount;
        private double averageRelevanceScore;
        private double altitudeToleranceFeet;
        private int recencyMinutes;
        private double corridorNauticalMiles;
        @Builder.Default
        private List<WeatherSourceSummary> relevant = new ArrayList<>();
        @Builder.Default
        private List<WeatherSourceSummary> excluded = new ArrayList<>();
    }

    @Data
    @Builder
    public static class CoordinationDraftSummary {
        private String id;
        private String missionId;
        private String reservationId;
        private String subject;
        private String family;
        private String direction;
        private String rawText;
        private String recommendedAction;
        @Builder.Default
        private List<String> recipients = new ArrayList<>();
        @Builder.Default
        private List<String> sourceRefs = new ArrayList<>();
    }

    @Data
    @Builder
    public static class PilotBriefSummary {
        private String missionId;
        private String missionNumber;
        private ZonedDateTime generatedAt;
        private MissionWeatherVerdictSummary verdict;
        private RouteImpactSummary routeImpact;
        private CoordinationDraftSummary coordinationDraft;
        @Builder.Default
        private List<WeatherSourceSummary> changes = new ArrayList<>();
        @Builder.Default
        private List<String> sourceSummaryLines = new ArrayList<>();
        private String decisionTraceSummary;
        private String printableText;
    }

    @Data
    @Builder
    public static class AffectedMissionSummary {
        private String missionId;
        private String missionNumber;
        private String status;
        private String action;
        private String priority;
        private double confidence;
        private int sourceCount;
        private int impactedSegmentCount;
        private int blockingConstraintCount;
        private String route;
        private String rationale;
        private ZonedDateTime lastObservedAt;
        private long ageSeconds;
        private boolean stale;
        private long guidanceLatencySeconds;
        private boolean guidanceTargetMet;
        @Builder.Default
        private List<String> sourceRefs = new ArrayList<>();
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
