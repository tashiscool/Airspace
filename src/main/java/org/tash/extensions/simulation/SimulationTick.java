package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationTick {
    private String id;
    private int minute;
    private ZonedDateTime simulatedTime;
    private String engineAction;
    private String recommendedAction;
    private double confidence;
    @Builder.Default
    private List<SimulatedAircraft> aircraft = new ArrayList<>();
    private AirportOpsTimeline airportOps;
    private SectorWorkloadModel sectorWorkload;
    private NationalDemandCapacitySnapshot nationalDemandCapacity;
    private WeatherEvolutionTick weatherEvolution;
    @Builder.Default
    private List<BehaviorStateMachine> behaviorStates = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private Map<String, String> replayHashes = new java.util.LinkedHashMap<>();
}
