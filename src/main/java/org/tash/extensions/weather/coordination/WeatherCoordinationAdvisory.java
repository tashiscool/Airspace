package org.tash.extensions.weather.coordination;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;

@Data
@Builder
public class WeatherCoordinationAdvisory {
    private final String id;
    private final WeatherDecisionAction action;
    private final WeatherRecommendedAction recommendedAction;
    private final WeatherDecisionSeverity severity;
    private final MeteorologistReviewPriority reviewPriority;
    private final String rationale;
}
