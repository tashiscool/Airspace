package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;

@Data
@Builder
public class HistoricalWeatherOutcome {
    private final String productId;
    private final String routeId;
    private final ZonedDateTime forecastTime;
    private final boolean routeBlocked;
    private final double observedDeviationRate;
    private final double observedCapacityImpact;
}
