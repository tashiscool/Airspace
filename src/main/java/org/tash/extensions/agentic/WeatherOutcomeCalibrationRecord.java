package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class WeatherOutcomeCalibrationRecord {
    private String productType;
    private double severityScore;
    private double echoTopsFeet;
    private double leadTimeHours;
    private double blockedOutcome;
}
