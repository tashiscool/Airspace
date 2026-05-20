package org.tash.extensions.weather.product;

public interface WeatherProductDecoder {
    WeatherProductType type();

    WeatherDecoderResult decode(String raw);
}
