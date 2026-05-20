package org.tash.extensions.weather.pirep;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;

@Data
@Builder
public class PirepDiagnostic {
    private final PirepDiagnosticType type;
    private final WeatherDecisionSeverity severity;
    private final String message;
}
