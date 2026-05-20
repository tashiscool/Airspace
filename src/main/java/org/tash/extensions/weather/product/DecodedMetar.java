package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class DecodedMetar {
    private final String stationId;
    private final ZonedDateTime reportTime;
    private final boolean corrected;
    private final Integer windDirectionDegrees;
    private final Double windSpeedKnots;
    private final Double windGustKnots;
    private final Integer variableWindFromDegrees;
    private final Integer variableWindToDegrees;
    private final Double visibilityStatuteMiles;
    private final Double runwayVisualRangeFeet;
    private final Double temperatureCelsius;
    private final Double dewpointCelsius;
    private final Double altimeterInchesHg;
    private final String remarks;
    @Builder.Default
    private final List<String> presentWeather = new ArrayList<>();
    @Builder.Default
    private final List<DecodedCloudLayer> cloudLayers = new ArrayList<>();

    public List<String> getPresentWeather() {
        return Collections.unmodifiableList(presentWeather == null ? Collections.emptyList() : presentWeather);
    }

    public List<DecodedCloudLayer> getCloudLayers() {
        return Collections.unmodifiableList(cloudLayers == null ? Collections.emptyList() : cloudLayers);
    }
}
