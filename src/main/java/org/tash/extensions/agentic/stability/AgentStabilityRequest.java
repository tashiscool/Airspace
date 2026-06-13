package org.tash.extensions.agentic.stability;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.agentic.AgentRunRequest;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentStabilityRequest {
    private AgentRunRequest agentRunRequest;
    private Integer iterations;
    private Double minAgreement;
    private Double minCitationJaccard;
    private Double maxCountCoefficientOfVariation;
}
