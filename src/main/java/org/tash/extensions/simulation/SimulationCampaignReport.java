package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationCampaignReport {
    private String id;
    private ZonedDateTime generatedAt;
    private int scenarioCount;
    private int passedScenarioCount;
    private SimulationKpiSummary aggregateKpis;
    @Builder.Default
    private List<SimulationRunResult> runs = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
