package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.engine.DecisionRuleApplication;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
public class RouteBlockagePrediction {
    private final ZonedDateTime forecastTime;
    private final Integer forecastHour;
    private final boolean blocked;
    private final WeatherDecisionSeverity severity;
    private final String rationale;
    private final WeatherRecommendedAction recommendedAction;
    private final String primaryHazardId;
    private final double confidence;
    private final double deviationLikelihood;
    private final double capacityImpact;
    private final double blockedProbability;
    private final String confidenceMath;
    private final String forecastSliceId;
    private final RouteImpactScoringBreakdown scoringBreakdown;
    private final double ensembleSpread;
    private final double confidenceIntervalLow;
    private final double confidenceIntervalHigh;
    @Builder.Default
    private final List<Integer> blockedSegmentIndexes = new ArrayList<>();
    @Builder.Default
    private final List<String> ruleIds = new ArrayList<>();
    @Builder.Default
    private final List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private final List<DecisionRuleApplication> ruleApplications = new ArrayList<>();
    @Builder.Default
    private final List<SectorCapacityImpact> sectorCapacityImpacts = new ArrayList<>();
    @Builder.Default
    private final Map<Integer, Double> segmentBlockedProbabilities = new LinkedHashMap<>();
    @Builder.Default
    private final List<RouteHazardIntersection> intersections = new ArrayList<>();

    public List<Integer> getBlockedSegmentIndexes() {
        return Collections.unmodifiableList(blockedSegmentIndexes);
    }

    public Map<Integer, Double> getSegmentBlockedProbabilities() {
        return Collections.unmodifiableMap(segmentBlockedProbabilities);
    }

    public List<String> getRuleIds() {
        return Collections.unmodifiableList(ruleIds);
    }

    public List<String> getSourceRefs() {
        return Collections.unmodifiableList(sourceRefs == null ? Collections.emptyList() : sourceRefs);
    }

    public List<DecisionRuleApplication> getRuleApplications() {
        return Collections.unmodifiableList(ruleApplications == null ? Collections.emptyList() : ruleApplications);
    }

    public List<SectorCapacityImpact> getSectorCapacityImpacts() {
        return Collections.unmodifiableList(sectorCapacityImpacts == null ? Collections.emptyList() : sectorCapacityImpacts);
    }

    public List<RouteHazardIntersection> getIntersections() {
        return Collections.unmodifiableList(intersections);
    }
}
