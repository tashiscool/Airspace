package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class ScenarioValidationResult {
    private boolean accepted;
    private String scenarioId;
    @Builder.Default
    private List<String> warnings = new ArrayList<>();
    @Builder.Default
    private List<String> errors = new ArrayList<>();
}
