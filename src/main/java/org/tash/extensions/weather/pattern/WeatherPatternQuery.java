package org.tash.extensions.weather.pattern;

import lombok.Data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
public class WeatherPatternQuery {
    private List<List<Double>> route = new ArrayList<>();
    private Double lowerAltitudeFeet;
    private Double upperAltitudeFeet;
    private Double corridorNauticalMiles;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;
}
