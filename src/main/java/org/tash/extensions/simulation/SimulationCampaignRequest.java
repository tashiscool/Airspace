package org.tash.extensions.simulation;

import lombok.Data;

import java.util.ArrayList;
import java.util.List;

@Data
public class SimulationCampaignRequest {
    private List<String> scenarioIds = new ArrayList<>();
    private boolean includeSensitivity;
    private String actor;
    private int iterationsPerScenario = 1;
    private int simulatedDayCount = 1;
    private long randomSeed = 20260620L;
    private List<CampaignKpiGate> kpiGates = new ArrayList<>();
}
