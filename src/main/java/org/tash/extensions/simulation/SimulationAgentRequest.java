package org.tash.extensions.simulation;

import lombok.Data;

import java.util.ArrayList;
import java.util.List;

@Data
public class SimulationAgentRequest {
    private String scenarioType;
    private String missionNumber;
    private String runId;
    private String campaignId;
    private boolean includeMalformedInputs;
    private int count = 1;
    private List<String> focusAreas = new ArrayList<>();
}
