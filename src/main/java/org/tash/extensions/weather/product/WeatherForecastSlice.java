package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class WeatherForecastSlice {
    private final int sequence;
    private final String groupType;
    private final String rawText;
    private final ZonedDateTime validStart;
    private final ZonedDateTime validEnd;
    private final Integer forecastHour;
    private final Double ceilingFeet;
    private final Double visibilityStatuteMiles;
    private final Double confidence;
    private final String sourceProductId;
    private final Double blockedProbability;
    @Builder.Default
    private final List<GeoCoordinate> geometry = new ArrayList<>();
    @Builder.Default
    private final List<GeoCoordinate> movementAdjustedGeometry = new ArrayList<>();
    @Builder.Default
    private final List<String> weatherPhenomena = new ArrayList<>();
    @Builder.Default
    private final List<String> ruleIds = new ArrayList<>();
    private final WeatherSourceSpan sourceSpan;

    public List<String> getWeatherPhenomena() {
        return Collections.unmodifiableList(weatherPhenomena == null ? Collections.emptyList() : weatherPhenomena);
    }

    public List<GeoCoordinate> getGeometry() {
        return Collections.unmodifiableList(geometry == null ? Collections.emptyList() : geometry);
    }

    public List<GeoCoordinate> getMovementAdjustedGeometry() {
        return Collections.unmodifiableList(movementAdjustedGeometry == null ? Collections.emptyList() : movementAdjustedGeometry);
    }

    public List<String> getRuleIds() {
        return Collections.unmodifiableList(ruleIds == null ? Collections.emptyList() : ruleIds);
    }
}
