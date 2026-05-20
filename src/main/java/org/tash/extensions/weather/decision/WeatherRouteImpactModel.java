package org.tash.extensions.weather.decision;

import java.util.List;

public class WeatherRouteImpactModel {
    private final WeatherDecisionSupportService decisionSupportService;

    public WeatherRouteImpactModel() {
        this(new WeatherDecisionSupportService());
    }

    public WeatherRouteImpactModel(WeatherDecisionSupportService decisionSupportService) {
        this.decisionSupportService = decisionSupportService == null
                ? new WeatherDecisionSupportService()
                : decisionSupportService;
    }

    public RouteWeatherImpactResult evaluate(RouteWeatherDecisionRequest request) {
        RouteWeatherAdvisory advisory = decisionSupportService.adviseRoute(request);
        List<RouteBlockagePrediction> predictions = decisionSupportService.predictRouteBlockage(request);
        return RouteWeatherImpactResult.builder()
                .advisory(advisory)
                .predictions(predictions)
                .maximumDeviationLikelihood(predictions.stream()
                        .mapToDouble(RouteBlockagePrediction::getDeviationLikelihood)
                        .max()
                        .orElse(0.0))
                .maximumCapacityImpact(predictions.stream()
                        .mapToDouble(RouteBlockagePrediction::getCapacityImpact)
                        .max()
                        .orElse(0.0))
                .maximumBlockedProbability(predictions.stream()
                        .mapToDouble(RouteBlockagePrediction::getBlockedProbability)
                        .max()
                        .orElse(0.0))
                .build();
    }
}
