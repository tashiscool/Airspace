package org.tash.extensions.engine;

import org.tash.extensions.weather.decision.RouteWeatherDecisionRequest;
import org.tash.extensions.weather.decision.RouteWeatherImpactResult;
import org.tash.extensions.weather.decision.WeatherRouteImpactModel;

/**
 * Engine-facing facade for evaluating route exposure to weather, PIREP-derived
 * hazards, NOTAM constraints, and future forecast slices.
 */
public class RouteHazardImpactService {
    private final WeatherRouteImpactModel impactModel;

    public RouteHazardImpactService() {
        this(new WeatherRouteImpactModel());
    }

    public RouteHazardImpactService(WeatherRouteImpactModel impactModel) {
        this.impactModel = impactModel == null ? new WeatherRouteImpactModel() : impactModel;
    }

    public RouteWeatherImpactResult evaluate(RouteWeatherDecisionRequest request) {
        return impactModel.evaluate(request == null ? RouteWeatherDecisionRequest.builder().build() : request);
    }
}
