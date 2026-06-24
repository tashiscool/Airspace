package org.tash.extensions.simulation;

import lombok.Data;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Data
public class SimulationRunRequest {
    private String scenarioId;
    private String historicalReplayDayId;
    private String actor;
    private boolean includeSensitivity;
    private int tickIntervalSeconds = 60;
    private int durationMinutes;
    private long randomSeed = 20260620L;
    private List<SimulatedAircraft> aircraftFleet = new ArrayList<>();
    private TrafficReplayBundle trafficReplay;
    private NationalDemandCapacityConfig nationalDemandCapacityConfig;
    private WeatherEnsembleConfig weatherEnsembleConfig;
    private SectorWorkloadModel sectorWorkloadModel;
    private AirportOpsTimeline airportOpsTimeline;
    private Map<String, Double> sensitivityOverrides = new LinkedHashMap<>();
}
