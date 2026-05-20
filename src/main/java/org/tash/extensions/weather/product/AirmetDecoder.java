package org.tash.extensions.weather.product;

import java.util.ArrayList;

public class AirmetDecoder extends AbstractWeatherProductDecoder {
    public AirmetDecoder(WeatherProductParser parser) {
        super(parser, WeatherProductType.AIRMET);
    }

    @Override
    protected Object decoded(WeatherProductParseResult result) {
        WeatherProduct product = result == null ? null : result.getProduct();
        if (product == null) {
            return null;
        }
        return DecodedSigmetAirmet.builder()
                .type(WeatherProductType.AIRMET)
                .hazard(product.getType().name())
                .lineWidthNauticalMiles(product.getLineWidthNauticalMiles())
                .lowerAltitudeFeet(product.getLowerAltitudeFeet())
                .upperAltitudeFeet(product.getUpperAltitudeFeet())
                .movement(product.getMovement())
                .echoTopFeet(product.getEchoTopFeet())
                .geometry(new ArrayList<>(product.getGeometry()))
                .build();
    }
}
