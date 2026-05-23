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
        private String notamType;
        private String notamAccountability;
        private String notamAffectedLocation;
        private String notamQCode;
        private String notamTraffic;
        private String notamPurpose;
        private String notamScope;
        private String notamLowerFlightLevel;
        private String notamUpperFlightLevel;
        private boolean notamEstimatedEnd;
        private boolean notamPermanentEnd;
        private boolean notamHasGeometry;
        private Double notamCenterLatitude;
        private Double notamCenterLongitude;
        private Double notamRadiusNauticalMiles;
        private String domesticNotamRecordType;
        private String domesticNotamKeyword;
        private String domesticNotamAccountability;
        private String domesticNotamLocation;
        private String domesticNotamNumber;
        private boolean domesticNotamUnofficial;
        private String domesticNotamQ23;
        private String domesticNotamQ45;
        private String domesticNotamSemanticFacilityFamily;
        private String domesticNotamSemanticCondition;
        private String domesticNotamSemanticAction;
        private String domesticNotamReducerRuleId;
        private String domesticNotamReducerName;
        private String serviceCommandType;
        private String serviceCommandService;
        private String serviceCommandDomain;
        private String serviceCommandOperation;
        private String serviceCommandRequestFormat;
        private String serviceCommandLocation;
        private String serviceCommandNotamId;
        private String serviceCommandRangeStart;
        private String serviceCommandRangeEnd;
        private String serviceCommandTableName;
        private boolean serviceCommandAccepted;
        private boolean serviceCommandCount;
        private boolean serviceCommandList;
        private boolean serviceCommandHistory;
        private boolean serviceCommandAll;
        private boolean serviceCommandCurrent;
        private boolean serviceCommandWmscrEcho;
        private boolean serviceCommandPrivilegedHistoryRequest;
        private String familySemantic;
        private String familyPreview;
        private String familyLifecycle;
        private String familyNotamAction;
        private String familySnowtamId;
        private String familyBirdtamId;
        private String familyAshtamId;
        private String familyGenotSeries;
        private String familyFdcId;
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
        private String missionId;
        private String reservationId;
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
        private double originalRouteDistanceNm;
        private double originalRouteEstimatedMinutes;
        private double originalRouteEstimatedFuelLb;
        private double originalRouteEstimatedCostUsd;
        private int impactedSegmentCount;
        private int blockingConstraintCount;
        @Builder.Default
        private List<String> impactedSegments = new ArrayList<>();
        @Builder.Default
        private List<String> sourceRefs = new ArrayList<>();
        @Builder.Default
        private List<String> avoidanceCandidates = new ArrayList<>();
        @Builder.Default
        private List<RouteCandidateComparisonSummary> candidateComparisons = new ArrayList<>();
        @Builder.Default
        private List<RerouteTraceSummary> whyRerouteTrace = new ArrayList<>();
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }

    @Data
    @Builder
    public static class RouteCandidateComparisonSummary {
        private String id;
        private String label;
        private String rationale;
        private double confidence;
        private RouteCostEstimateSummary cost;
        @Builder.Default
        private List<String> routePointLabels = new ArrayList<>();
        @Builder.Default
        private List<ConstraintImpactSummary> avoidedConstraints = new ArrayList<>();
        @Builder.Default
        private List<ConstraintImpactSummary> residualConstraints = new ArrayList<>();
        @Builder.Default
        private List<String> sourceRefs = new ArrayList<>();
        @Builder.Default
        private List<RerouteTraceSummary> trace = new ArrayList<>();
    }

    @Data
    @Builder
    public static class RouteCostEstimateSummary {
        private double distanceNm;
        private double additionalDistanceNm;
        private double estimatedMinutes;
        private double additionalMinutes;
        private double estimatedFuelLb;
        private double additionalFuelLb;
        private double estimatedCostUsd;
        private double additionalCostUsd;
        private double cruiseSpeedKnots;
        private double fuelBurnLbPerNm;
        private double fuelCostUsdPerLb;
        private double delayCostUsdPerMinute;
    }

    @Data
    @Builder
    public static class ConstraintImpactSummary {
        private String id;
        private String family;
        private String label;
        private String severity;
        private String sourceRef;
        private String rationale;
    }

    @Data
    @Builder
    public static class RerouteTraceSummary {
        private String stage;
        private String ruleId;
        private String message;
        private String sourceRef;
        private double confidence;
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
        private int rerouteCandidateCount;
        private String bestCandidateLabel;
        private double rerouteAdditionalDistanceNm;
        private double rerouteAdditionalMinutes;
        private double rerouteAdditionalFuelLb;
        private double rerouteAdditionalCostUsd;
        private int avoidedConstraintCount;
        private int residualConstraintCount;
        @Builder.Default
        private List<String> sourceRefs = new ArrayList<>();
        @Builder.Default
        private List<List<Double>> routeCoordinates = new ArrayList<>();
    }

    @Data
    @Builder
    public static class DecisionSummary {
        private String id;
        private String action;
        private String recommendedAction;
        private double confidence;
        private String rationale;
        private RouteImpactSummary routeImpact;
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
