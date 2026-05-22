package org.tash.extensions.agentic;

import lombok.Data;

@Data
public class ScenarioFixtureRequest {
    private String scenarioType;
    private String missionNumber;
    private boolean includeMalformedInputs;
}
