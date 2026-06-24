package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class WeatherEnsembleConfig {
    private int memberCount;
    private long randomSeed;
    private double movementSpeedKnots;
    private double forecastConfidence;
    private double growthRate;
    private double decayRate;
    private String assumptions;
}
