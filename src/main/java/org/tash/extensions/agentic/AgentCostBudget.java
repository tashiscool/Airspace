package org.tash.extensions.agentic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentCostBudget {
    private double maxCostUsd;
    private double estimatedCostUsd;
    private int timeoutMillis;
    private int retryCap;
    private boolean circuitBreakerArmed;
    private String fallbackMode;
}
