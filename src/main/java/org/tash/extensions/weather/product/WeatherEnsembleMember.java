package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class WeatherEnsembleMember {
    private final String id;
    private final double weight;
    private final WeatherProduct product;
}
