package org.tash.extensions.weather.product;

public class PirepDecoder extends AbstractWeatherProductDecoder {
    public PirepDecoder(WeatherProductParser parser) {
        super(parser, WeatherProductType.PIREP_DERIVED);
    }

    @Override
    protected Object decoded(WeatherProductParseResult result) {
        return result == null ? null : result.getPirepReport();
    }
}
