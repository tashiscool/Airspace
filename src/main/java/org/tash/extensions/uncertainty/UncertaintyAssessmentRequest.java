package org.tash.extensions.uncertainty;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.product.WeatherProduct;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder(toBuilder = true)
public class UncertaintyAssessmentRequest {
    @Builder.Default
    private final List<GeoCoordinate> route = new ArrayList<>();
    @Builder.Default
    private final List<WeatherProduct> weatherProducts = new ArrayList<>();
    private final PositionUncertaintyModel positionUncertainty;
    private final ForecastUncertaintyModel forecastUncertainty;
    private final ZonedDateTime decisionTime;
    @Builder.Default
    private final double baseConfidence = 1.0;

    public List<GeoCoordinate> getRoute() {
        return Collections.unmodifiableList(route == null ? Collections.emptyList() : route);
    }
}
