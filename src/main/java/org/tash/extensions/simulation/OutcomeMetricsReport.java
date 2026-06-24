package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class OutcomeMetricsReport {
    private String id;
    private ZonedDateTime generatedAt;
    private String scope;
    private String sourceMode;
    private String runId;
    private String campaignId;
    private String scenarioId;
    private double baselineDelayMinutes;
    private double mitigatedDelayMinutes;
    private double delayMinutesSaved;
    private double rerouteMiles;
    private double additionalFuelPounds;
    private double holdingFuelSavedPounds;
    private double fuelImpactPounds;
    private int sectorOverloadAvoidedCount;
    private int falseClearCount;
    private int falseBlockCount;
    private double sourceRefCompletenessRate;
    private long operatorTimeToDecisionSeconds;
    private int rerouteCandidateCount;
    private int routeAlternativeCount;
    private int affectedFlightCount;
    private int overloadedAirportCount;
    private int overloadedSectorCount;
    private int proposedTmiCount;
    @Builder.Default
    private List<OutcomeMetricSummary> metrics = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> assumptions = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
