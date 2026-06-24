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
public class SimulationReplayBundle {
    private String id;
    private String runId;
    private String scenarioId;
    private String finalAction;
    private ZonedDateTime generatedAt;
    private SimulationWorldState worldState;
    @Builder.Default
    private List<SimulationStepResult> steps = new ArrayList<>();
    @Builder.Default
    private Map<String, String> replayHashes = new java.util.LinkedHashMap<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
