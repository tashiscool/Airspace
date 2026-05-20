package org.tash.extensions.uncertainty;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class ForecastUncertaintyModel {
    private final double forecastSpreadNauticalMiles;
    private final double confidencePenalty;
    private final String source;
}
