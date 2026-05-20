package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;

import java.time.ZonedDateTime;
import org.tash.extensions.weather.product.WeatherMovementVector;
import org.tash.extensions.weather.product.WeatherProductType;

@Data
@Builder
public class RouteHazardIntersection {
    private final String hazardId;
    private final String productId;
    private final WeatherElementType hazardType;
    private final HazardSeverity hazardSeverity;
    private final int segmentIndex;
    private final ZonedDateTime estimatedEntryTime;
    private final ZonedDateTime estimatedExitTime;
    private final GeoCoordinate estimatedEntryPoint;
    private final GeoCoordinate estimatedExitPoint;
    private final double confidence;
    private final WeatherProductStatus productStatus;
    private final String sourceProduct;
    private final String provenance;
    private final Integer forecastHour;
    private final WeatherProductType productType;
    private final WeatherMovementVector movement;
    private final Double echoTopFeet;
    private final Double growthTrend;
    private final String stormPhase;
    private final WeatherRecommendedAction recommendedAction;
    private final String rationale;
}
