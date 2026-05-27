package org.tash.extensions.weather.pattern;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
public class WeatherPattern {
    private String id;
    private WeatherPatternType type;
    private String productFamily;
    private String sourceFamily;
    private String sourceProductId;
    private String sourceUrl;
    private String rawText;
    private String geometryIntent;
    private ZonedDateTime issuedAt;
    private ZonedDateTime receivedAt;
    private ZonedDateTime validStart;
    private ZonedDateTime validEnd;
    private Integer forecastHour;
    private Double lowerAltitudeFeet;
    private Double upperAltitudeFeet;
    private Double movementBearingDegrees;
    private Double movementSpeedKnots;
    private String severity;
    private double confidence;
    private String freshnessCategory;
    private String rationale;
    @Builder.Default
    private List<GeoCoordinate> geometry = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
