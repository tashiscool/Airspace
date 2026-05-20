package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;

@Data
@Builder
public class WeatherProductDiagnostic {
    private final WeatherDiagnosticType type;
    private final WeatherDecisionSeverity severity;
    private final String message;
}
