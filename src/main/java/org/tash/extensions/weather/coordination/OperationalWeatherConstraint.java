package org.tash.extensions.weather.coordination;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class OperationalWeatherConstraint {
    private final String id;
    private final String primaryHazardId;
    private final ZonedDateTime effectiveStart;
    private final ZonedDateTime effectiveEnd;
    @Builder.Default
    private final List<Integer> impactedSegmentIndexes = new ArrayList<>();
    private final WeatherRecommendedAction recommendedAction;
    private final String rationale;

    public List<Integer> getImpactedSegmentIndexes() {
        return Collections.unmodifiableList(impactedSegmentIndexes);
    }
}
