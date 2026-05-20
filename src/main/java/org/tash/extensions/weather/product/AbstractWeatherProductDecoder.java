package org.tash.extensions.weather.product;

abstract class AbstractWeatherProductDecoder implements WeatherProductDecoder {
    private final WeatherProductParser parser;
    private final WeatherProductType type;

    AbstractWeatherProductDecoder(WeatherProductParser parser, WeatherProductType type) {
        this.parser = parser;
        this.type = type;
    }

    @Override
    public WeatherProductType type() {
        return type;
    }

    @Override
    public WeatherDecoderResult decode(String raw) {
        WeatherProductParseResult result = parser.parseResolved(raw, type);
        return WeatherDecoderResult.builder()
                .parseResult(result)
                .decoded(decoded(result))
                .build();
    }

    protected Object decoded(WeatherProductParseResult result) {
        return result == null ? null : result.getProduct();
    }
}
