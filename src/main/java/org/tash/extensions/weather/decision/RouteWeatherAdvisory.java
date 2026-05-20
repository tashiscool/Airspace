package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class RouteWeatherAdvisory {
    private final WeatherDecisionAction action;
    private final WeatherRecommendedAction recommendedAction;
    private final WeatherDecisionSeverity severity;
    private final String rationale;
    private final double confidence;
    private final String primaryHazardId;
    @Builder.Default
    private final List<RouteHazardIntersection> intersections = new ArrayList<>();
    @Builder.Default
    private final List<ImpactedRouteSegment> impactedSegments = new ArrayList<>();
    @Builder.Default
    private final List<String> warnings = new ArrayList<>();
    @Builder.Default
    private final List<GeoCoordinate> suggestedPath = new ArrayList<>();
    @Builder.Default
    private final List<RouteBlockagePrediction> blockagePredictions = new ArrayList<>();

    public boolean isOperationallyClear() {
        return action == WeatherDecisionAction.CLEAR || action == WeatherDecisionAction.MONITOR;
    }

    public List<RouteHazardIntersection> getIntersections() {
        return Collections.unmodifiableList(intersections);
    }

    public List<ImpactedRouteSegment> getImpactedSegments() {
        return Collections.unmodifiableList(impactedSegments);
    }

    public List<String> getWarnings() {
        return Collections.unmodifiableList(warnings);
    }

    public List<GeoCoordinate> getSuggestedPath() {
        return Collections.unmodifiableList(suggestedPath);
    }

    public List<RouteBlockagePrediction> getBlockagePredictions() {
        return Collections.unmodifiableList(blockagePredictions);
    }
}
