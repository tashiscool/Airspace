package org.tash.extensions.weather.product;

import java.util.ArrayList;

public class SigmetDecoder extends AbstractWeatherProductDecoder {
    public SigmetDecoder(WeatherProductParser parser) {
        super(parser, WeatherProductType.SIGMET);
    }

    @Override
    protected Object decoded(WeatherProductParseResult result) {
        WeatherProduct product = result == null ? null : result.getProduct();
        if (product == null) {
            return null;
        }
        return DecodedSigmetAirmet.builder()
                .type(WeatherProductType.SIGMET)
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
