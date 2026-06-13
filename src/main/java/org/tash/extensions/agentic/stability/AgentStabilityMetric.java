package org.tash.extensions.agentic.stability;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentStabilityMetric {
    private String id;
    private String name;
    private double value;
    private double threshold;
    private boolean accepted;
    private String details;
}
