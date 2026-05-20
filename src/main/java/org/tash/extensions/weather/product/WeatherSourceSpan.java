package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class WeatherSourceSpan {
    private final String field;
    private final int startOffset;
    private final int endOffset;
    private final String text;
}
