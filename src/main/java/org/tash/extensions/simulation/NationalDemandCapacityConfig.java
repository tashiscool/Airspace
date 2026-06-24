package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class NationalDemandCapacityConfig {
    private String id;
    @Builder.Default
    private int flightCount = 1000;
    @Builder.Default
    private int airportCount = 12;
    @Builder.Default
    private int sectorCount = 24;
    @Builder.Default
    private int durationMinutes = 180;
    @Builder.Default
    private int tickIntervalMinutes = 15;
    @Builder.Default
    private long randomSeed = 20260623L;
    @Builder.Default
    private double demandSpikeFactor = 1.35;
    @Builder.Default
    private double capacityReductionFactor = 0.72;
    @Builder.Default
    private boolean includeWeatherCapacityReduction = true;
    @Builder.Default
    private List<String> airportIds = new ArrayList<>();
    @Builder.Default
    private List<String> sectorIds = new ArrayList<>();
    @Builder.Default
    private String sourceMode = "LOCAL_SYNTHETIC_NAS_SCALE";
}
