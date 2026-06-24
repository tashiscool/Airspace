package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationScenario {
    private String id;
    private String name;
    private String capabilityStory;
    private String narrative;
    private String missionNumber;
    private String carfAltrv;
    private String expectedFinalAction;
    private String expectedGuidance;
    @Builder.Default
    private List<List<Double>> route = new ArrayList<>();
    @Builder.Default
    private List<SimulationEvent> events = new ArrayList<>();
    @Builder.Default
    private List<String> expectedSourceFamilies = new ArrayList<>();
    @Builder.Default
    private Map<String, Double> sensitivityDefaults = new LinkedHashMap<>();
}
