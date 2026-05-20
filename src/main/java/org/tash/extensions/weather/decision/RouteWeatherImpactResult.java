package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class RouteWeatherImpactResult {
    private final RouteWeatherAdvisory advisory;
    @Builder.Default
    private final List<RouteBlockagePrediction> predictions = new ArrayList<>();
    private final double maximumDeviationLikelihood;
    private final double maximumCapacityImpact;
    private final double maximumBlockedProbability;

    public List<RouteBlockagePrediction> getPredictions() {
        return Collections.unmodifiableList(predictions);
    }
}
