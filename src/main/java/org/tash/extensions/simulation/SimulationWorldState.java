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
public class SimulationWorldState {
    private String runId;
    private String scenarioId;
    private int tickIntervalSeconds;
    private int durationMinutes;
    private long randomSeed;
    private ZonedDateTime generatedAt;
    private NationalDemandCapacityReport nationalDemandCapacityReport;
    @Builder.Default
    private List<SimulationTick> ticks = new ArrayList<>();
    @Builder.Default
    private List<String> assumptions = new ArrayList<>();
}
