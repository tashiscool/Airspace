package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class WeatherConfidence {
    private final double value;
    private final String basis;

    public double normalized() {
        return Math.max(0.0, Math.min(1.0, value));
    }

    public boolean isLow() {
        return normalized() > 0.0 && normalized() < 0.5;
    }
}
