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
public class SimulationRunResult {
    private String id;
    private String scenarioId;
    private String scenarioName;
    private String missionId;
    private String reservationId;
    private String missionNumber;
    private ZonedDateTime startedAt;
    private ZonedDateTime completedAt;
    private String finalAction;
    private String expectedFinalAction;
    private SimulationKpiSummary kpiSummary;
    private SimulationWorldState worldState;
    private SimulationReplayBundle replayBundle;
    private NationalDemandCapacityReport nationalDemandCapacityReport;
    @Builder.Default
    private List<SimulationStepResult> steps = new ArrayList<>();
    @Builder.Default
    private Map<String, Double> sensitivity = new java.util.LinkedHashMap<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
