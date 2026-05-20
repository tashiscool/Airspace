package org.tash.extensions.weather.product;

public class CwapDecoder extends AbstractWeatherProductDecoder {
    public CwapDecoder(WeatherProductParser parser) {
        super(parser, WeatherProductType.NEXRAD_POLYGON);
    }
}
