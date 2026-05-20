package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;

import java.time.ZonedDateTime;

@Data
@Builder
public class ImpactedRouteSegment {
    private final int segmentIndex;
    private final GeoCoordinate segmentStart;
    private final GeoCoordinate segmentEnd;
    private final ZonedDateTime estimatedEntryTime;
    private final ZonedDateTime estimatedExitTime;
    private final String primaryHazardId;
    private final WeatherElementType primaryHazardType;
    private final HazardSeverity primaryHazardSeverity;
    private final double confidence;
    private final WeatherRecommendedAction recommendedAction;
    private final String rationale;
}
