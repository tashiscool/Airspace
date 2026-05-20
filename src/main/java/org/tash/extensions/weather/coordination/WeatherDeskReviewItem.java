package org.tash.extensions.weather.coordination;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;

@Data
@Builder
public class WeatherDeskReviewItem {
    private final String id;
    private final MeteorologistReviewPriority priority;
    private final ZonedDateTime createdAt;
    private final String subject;
    private final String rationale;
    private final String relatedHazardId;
}
