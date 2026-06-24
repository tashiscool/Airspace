package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationWeatherEvolutionState {
    private String evolutionModel;
    private int ensembleMemberCount;
    private int forecastHour;
    private double movementSpeedKnots;
    private double growthRate;
    private double decayRate;
    private double meanBlockedProbability;
    private double probabilitySpread;
    private String stormPhase;
    private String rationale;
}
