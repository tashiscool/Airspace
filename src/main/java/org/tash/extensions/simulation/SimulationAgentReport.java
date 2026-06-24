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
public class SimulationAgentReport {
    private String id;
    private String agentType;
    private ZonedDateTime generatedAt;
    @Builder.Default
    private List<ScenarioBundle> generatedScenarioDrafts = new ArrayList<>();
    @Builder.Default
    private List<String> findings = new ArrayList<>();
    @Builder.Default
    private List<String> citations = new ArrayList<>();
    @Builder.Default
    private List<String> policyGuards = new ArrayList<>();
}
