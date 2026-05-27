package org.tash.extensions.weather.pattern;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder
public class RouteWeatherPatternIntersection {
    private String patternId;
    private WeatherPatternType patternType;
    private String severity;
    private double confidence;
    private boolean timeOverlap;
    private boolean altitudeOverlap;
    private boolean geometryOverlap;
    private int segmentIndex;
    private double nearestDistanceNauticalMiles;
    private String rationale;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
