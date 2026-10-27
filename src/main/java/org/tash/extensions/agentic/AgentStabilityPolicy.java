package org.tash.extensions.agentic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentStabilityPolicy {
    @Builder.Default
    private boolean automatic = true;
    @Builder.Default
    private AgentStabilityMode deterministicMode = AgentStabilityMode.WARN_ONLY;
    @Builder.Default
    private AgentStabilityMode modelBackedMode = AgentStabilityMode.ENFORCE;
    @Builder.Default
    private int iterations = 2;
    @Builder.Default
    private double minAgreement = 1.0;
    @Builder.Default
    private double minCitationJaccard = 0.95;
    @Builder.Default
    private double maxCountCoefficientOfVariation = 0.0;
}
