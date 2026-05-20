package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;

@Data
@Builder
public class DecodedForecastGroup {
    private final String groupType;
    private final String rawText;
    private final ZonedDateTime validStart;
    private final ZonedDateTime validEnd;
    private final Double confidence;
    private final Double ceilingFeet;
    private final Double visibilityStatuteMiles;
}
