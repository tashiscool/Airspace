package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder
public class WeatherCoordinationDraftRequest {
    private final String missionId;
    private final String hazardOrDecisionId;
    private final WeatherRecommendedAction recommendedAction;
    private final String impactSummary;
    @Builder.Default private final List<DecisionSourceRef> sourceRefs = new ArrayList<>();
    @Builder.Default private final List<String> recipients = new ArrayList<>();
}
