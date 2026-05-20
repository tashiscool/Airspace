package org.tash.extensions.weather.coordination;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;

import java.time.ZonedDateTime;

@Data
@Builder
public class ControllerHandoffNote {
    private final String id;
    private final ZonedDateTime createdAt;
    private final String routeId;
    private final WeatherRecommendedAction recommendedAction;
    private final double confidence;
    private final String rationale;
}
