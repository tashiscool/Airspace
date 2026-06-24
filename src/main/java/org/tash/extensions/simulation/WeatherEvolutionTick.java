package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class WeatherEvolutionTick {
    private int minute;
    private WeatherEnsembleConfig config;
    @Builder.Default
    private List<WeatherEnsembleMember> members = new ArrayList<>();
    private double meanBlockedProbability;
    private double probabilitySpread;
}
