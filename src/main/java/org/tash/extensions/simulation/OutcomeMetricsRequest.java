package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class OutcomeMetricsRequest {
    private String runId;
    private String campaignId;
    private String scenarioId;
    @Builder.Default
    private boolean runSimulation = false;
    private SimulationRunRequest simulationRunRequest;
    private NationalDemandCapacityConfig demandCapacityConfig;
    @Builder.Default
    private boolean includeTfmBoard = true;
}
