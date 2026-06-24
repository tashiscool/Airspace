package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class ScenarioBundle {
    private String id;
    private String name;
    private String airportId;
    private String region;
    private String useCase;
    private SimulationScenario scenario;
    private TrafficFlowScenario trafficFlow;
    private TrafficReplayBundle trafficReplay;
    private AirportOpsTimeline airportOps;
    private WeatherEnsembleConfig weatherEnsembleConfig;
    @Builder.Default
    private List<CampaignKpiGate> kpiGates = new ArrayList<>();
    @Builder.Default
    private Map<String, String> expectedSummary = new java.util.LinkedHashMap<>();
}
