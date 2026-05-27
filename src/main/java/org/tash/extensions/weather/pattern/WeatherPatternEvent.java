package org.tash.extensions.weather.pattern;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder
public class WeatherPatternEvent {
    private String id;
    private WeatherPatternType type;
    private String label;
    private String severity;
    private double confidence;
    private ZonedDateTime validStart;
    private ZonedDateTime validEnd;
    private int affectedMissionCount;
    private int productCount;
    private int pirepCount;
    private String rationale;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> patternIds = new ArrayList<>();
    @Builder.Default
    private List<GeoCoordinate> representativeGeometry = new ArrayList<>();
}
