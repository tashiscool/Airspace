package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class NationalDemandCapacityReport {
    private String id;
    private String sourceMode;
    private int flightCount;
    private int airportCount;
    private int sectorCount;
    private int durationMinutes;
    private int tickIntervalMinutes;
    private double peakAirportDemandCapacityRatio;
    private double peakSectorDemandCapacityRatio;
    private int peakOverloadedAirportCount;
    private int peakOverloadedSectorCount;
    private int totalTmiRecommendationCount;
    private TrafficReplayBundle trafficReplay;
    @Builder.Default
    private List<NationalDemandCapacitySnapshot> snapshots = new ArrayList<>();
    @Builder.Default
    private List<String> assumptions = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
