package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.product.WeatherProductType;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class StormCellObservation {
    private final String observationId;
    private final WeatherProductType productType;
    private final ZonedDateTime observedAt;
    private final Double echoTopFeet;
    private final Double growthTrend;
    private final String stormPhase;
    @Builder.Default
    private final List<GeoCoordinate> geometry = new ArrayList<>();

    public List<GeoCoordinate> getGeometry() {
        return Collections.unmodifiableList(geometry == null ? Collections.emptyList() : geometry);
    }
}
