package org.tash.extensions.uncertainty;

import java.util.ArrayList;
import java.util.List;

public class DecisionUncertaintyService {
    public UncertaintyAssessmentResult assess(UncertaintyAssessmentRequest request) {
        UncertaintyAssessmentRequest safe = request == null
                ? UncertaintyAssessmentRequest.builder().build()
                : request;
        double penalty = 0.0;
        double expandedCorridor = 0.0;
        List<String> warnings = new ArrayList<>();
        if (safe.getPositionUncertainty() != null) {
            PositionUncertaintyModel position = safe.getPositionUncertainty();
            expandedCorridor += Math.max(0.0, position.getHorizontalNauticalMiles());
            penalty += Math.min(0.25, position.getHorizontalNauticalMiles() / 120.0);
            penalty += Math.min(0.15, position.getVerticalFeet() / 40000.0);
            penalty += Math.max(0.0, 1.0 - position.getConfidence()) * 0.2;
            if (position.getHorizontalNauticalMiles() > 10.0) {
                warnings.add("High aircraft position uncertainty");
            }
        }
        if (safe.getForecastUncertainty() != null) {
            ForecastUncertaintyModel forecast = safe.getForecastUncertainty();
            expandedCorridor += Math.max(0.0, forecast.getForecastSpreadNauticalMiles());
            penalty += Math.min(0.25, forecast.getForecastSpreadNauticalMiles() / 160.0);
            penalty += Math.max(0.0, forecast.getConfidencePenalty());
            if (forecast.getForecastSpreadNauticalMiles() > 20.0) {
                warnings.add("High forecast spatial uncertainty");
            }
        }
        penalty = Math.min(0.6, penalty);
        double adjusted = Math.max(0.0, Math.min(1.0, safe.getBaseConfidence() - penalty));
        return UncertaintyAssessmentResult.builder()
                .adjustedConfidence(adjusted)
                .confidencePenalty(penalty)
                .expandedCorridorNauticalMiles(expandedCorridor)
                .rationale("Decision confidence adjusted for position and forecast uncertainty")
                .warnings(warnings)
                .build();
    }
}
