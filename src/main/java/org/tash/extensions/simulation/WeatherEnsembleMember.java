package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class WeatherEnsembleMember {
    private String id;
    private double latitudeOffset;
    private double longitudeOffset;
    private double blockedProbability;
    private double confidence;
    private String stormPhase;
}
