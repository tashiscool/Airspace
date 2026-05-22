package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.LinkedHashMap;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class ScenarioFixtureBundle {
    private String id;
    private String scenarioType;
    private String missionNumber;
    private String carfAltrv;
    @Builder.Default
    private List<String> usnsMessages = new ArrayList<>();
    @Builder.Default
    private List<String> weatherMessages = new ArrayList<>();
    @Builder.Default
    private List<String> pireps = new ArrayList<>();
    @Builder.Default
    private List<String> notams = new ArrayList<>();
    @Builder.Default
    private List<List<Double>> route = new ArrayList<>();
    @Builder.Default
    private Map<String, String> expectedSummary = new LinkedHashMap<>();
}
