package org.tash.extensions.weather.product;

import org.tash.extensions.weather.WeatherElementType;

public enum WeatherProductType {
    TURBULENCE(WeatherElementType.TURBULENCE),
    ICING(WeatherElementType.ICING),
    CONVECTION(WeatherElementType.CONVECTION),
    CEILING(WeatherElementType.CEILING),
    VISIBILITY(WeatherElementType.VISIBILITY),
    SIGMET(null),
    AIRMET(null),
    METAR(null),
    TAF(null),
    NEXRAD_POLYGON(WeatherElementType.CONVECTION),
    PIREP_DERIVED(null),
    GENERIC_FORECAST_HAZARD(null);

    private final WeatherElementType weatherElementType;

    WeatherProductType(WeatherElementType weatherElementType) {
        this.weatherElementType = weatherElementType;
    }

    public WeatherElementType getWeatherElementType() {
        return weatherElementType;
    }
}
