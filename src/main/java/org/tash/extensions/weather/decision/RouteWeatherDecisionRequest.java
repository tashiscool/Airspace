package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.EnsembleWeatherProduct;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class RouteWeatherDecisionRequest {
    private static final Duration DEFAULT_ROUTE_DURATION = Duration.ofMinutes(60);
    private static final Duration DEFAULT_FORECAST_HORIZON = Duration.ofHours(8);
    private static final Duration DEFAULT_FORECAST_STEP = Duration.ofMinutes(15);

    @Builder.Default
    private final List<GeoCoordinate> route = new ArrayList<>();
    private final ZonedDateTime departureTime;
    private final Duration routeDuration;
    private final Duration forecastHorizon;
    private final Duration forecastStep;
    @Builder.Default
    private final List<WeatherProduct> products = new ArrayList<>();
    @Builder.Default
    private final List<EnsembleWeatherProduct> ensembleProducts = new ArrayList<>();
    @Builder.Default
    private final List<WeatherHazardSnapshot> hazards = new ArrayList<>();
    private final RouteImpactCalibrationModel calibrationModel;
    @Builder.Default
    private final List<SectorDemandSnapshot> sectorDemand = new ArrayList<>();
    @Builder.Default
    private final List<RouteDemandSnapshot> routeDemand = new ArrayList<>();

    public List<GeoCoordinate> route() {
        return Collections.unmodifiableList(route);
    }

    public Duration routeDurationOrDefault() {
        return routeDuration == null || routeDuration.isZero() || routeDuration.isNegative()
                ? DEFAULT_ROUTE_DURATION
                : routeDuration;
    }

    public Duration forecastHorizonOrDefault() {
        return forecastHorizon == null || forecastHorizon.isZero() || forecastHorizon.isNegative()
                ? DEFAULT_FORECAST_HORIZON
                : forecastHorizon;
    }

    public Duration forecastStepOrDefault() {
        return forecastStep == null || forecastStep.isZero() || forecastStep.isNegative()
                ? DEFAULT_FORECAST_STEP
                : forecastStep;
    }

    public List<WeatherProduct> products() {
        return Collections.unmodifiableList(products);
    }

    public List<EnsembleWeatherProduct> ensembleProducts() {
        return Collections.unmodifiableList(ensembleProducts);
    }

    public List<WeatherHazardSnapshot> hazards() {
        return Collections.unmodifiableList(hazards);
    }

    public RouteImpactCalibrationModel calibrationOrDefault() {
        return calibrationModel == null ? RouteImpactCalibrationModel.builder().build() : calibrationModel;
    }

    public List<SectorDemandSnapshot> sectorDemand() {
        return Collections.unmodifiableList(sectorDemand);
    }

    public List<RouteDemandSnapshot> routeDemand() {
        return Collections.unmodifiableList(routeDemand);
    }
}
