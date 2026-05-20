package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class WeatherDecoderResult {
    private final WeatherProductParseResult parseResult;
    private final Object decoded;

    public boolean isAccepted() {
        return parseResult != null && parseResult.isAccepted();
    }

    public boolean isClassifiedOnly() {
        return parseResult != null && parseResult.isClassifiedOnly();
    }
}
