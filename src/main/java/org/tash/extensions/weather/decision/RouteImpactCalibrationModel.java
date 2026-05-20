package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.product.WeatherProductType;

import java.util.EnumMap;
import java.util.Map;

@Data
@Builder(toBuilder = true)
public class RouteImpactCalibrationModel {
    @Builder.Default
    private final String version = "default-route-impact-calibration-2026-05-20";
    @Builder.Default
    private final double leadTimeDecayPerHour = 0.08;
    @Builder.Default
    private final double minimumLeadTimeConfidence = 0.25;
    @Builder.Default
    private final double growthMultiplier = 0.15;
    @Builder.Default
    private final double blockedThreshold = 0.80;
    @Builder.Default
    private final double probabilityScale = 0.95;
    @Builder.Default
    private final double multiIntersectionCapacityBoost = 0.05;
    @Builder.Default
    private final double maxCapacityBoost = 0.20;
    @Builder.Default
    private final Map<HazardSeverity, Double> severityWeights = defaultSeverityWeights();
    @Builder.Default
    private final Map<WeatherProductType, Double> productTypeWeights = defaultProductTypeWeights();

    public double severityWeight(HazardSeverity severity) {
        return severityWeights.getOrDefault(severity, 0.2);
    }

    public double productWeight(WeatherProductType type) {
        return type == null ? 1.0 : productTypeWeights.getOrDefault(type, 1.0);
    }

    public static RouteImpactCalibrationModel load(CalibrationDataset dataset) {
        if (dataset == null || dataset.outcomes().isEmpty()) {
            return RouteImpactCalibrationModel.builder().build();
        }
        double blocked = 0.0;
        double deviation = 0.0;
        double capacity = 0.0;
        for (HistoricalWeatherOutcome outcome : dataset.outcomes()) {
            if (outcome.isRouteBlocked()) {
                blocked += 1.0;
            }
            deviation += outcome.getObservedDeviationRate();
            capacity += outcome.getObservedCapacityImpact();
        }
        double count = dataset.outcomes().size();
        return RouteImpactCalibrationModel.builder()
                .version("loaded-" + dataset.getId())
                .blockedThreshold(clamp(0.55 + (blocked / count) * 0.35))
                .probabilityScale(clamp(0.65 + (deviation / count) * 0.35))
                .multiIntersectionCapacityBoost(clamp((capacity / count) * 0.10))
                .maxCapacityBoost(clamp(0.10 + (capacity / count) * 0.25))
                .build();
    }

    private static Map<HazardSeverity, Double> defaultSeverityWeights() {
        Map<HazardSeverity, Double> weights = new EnumMap<>(HazardSeverity.class);
        weights.put(HazardSeverity.EXTREME, 1.0);
        weights.put(HazardSeverity.SEVERE, 0.75);
        weights.put(HazardSeverity.MODERATE, 0.45);
        weights.put(HazardSeverity.LIGHT, 0.20);
        return weights;
    }

    private static Map<WeatherProductType, Double> defaultProductTypeWeights() {
        Map<WeatherProductType, Double> weights = new EnumMap<>(WeatherProductType.class);
        weights.put(WeatherProductType.NEXRAD_POLYGON, 1.08);
        weights.put(WeatherProductType.SIGMET, 1.05);
        weights.put(WeatherProductType.AIRMET, 0.85);
        weights.put(WeatherProductType.PIREP_DERIVED, 0.90);
        weights.put(WeatherProductType.TAF, 0.70);
        weights.put(WeatherProductType.METAR, 0.60);
        return weights;
    }

    private static double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
