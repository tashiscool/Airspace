package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.List;

@ApplicationScoped
public class AgentCitationValidator {
    public List<String> validate(AgentRunResult result, AgentPolicy policy) {
        AgentPolicy safePolicy = policy == null ? AgentPolicy.builder().build() : policy;
        List<String> diagnostics = new ArrayList<>();
        if (!safePolicy.isRequireCitationsForOperationalClaims()) {
            return diagnostics;
        }
        if (result == null) {
            diagnostics.add("Agent result is missing");
            return diagnostics;
        }
        for (AgentFinding finding : result.getFindings()) {
            if (finding.getCitations() == null || finding.getCitations().isEmpty()) {
                diagnostics.add("Finding lacks source citation: " + finding.getId());
            }
        }
        for (AgentRecommendation recommendation : result.getRecommendations()) {
            if (recommendation.getCitations() == null || recommendation.getCitations().isEmpty()) {
                diagnostics.add("Recommendation lacks source citation: " + recommendation.getId());
            }
        }
        for (AgentTask task : result.getTasks()) {
            if (task.getCitations() == null || task.getCitations().isEmpty()) {
                diagnostics.add("Task lacks source citation: " + task.getId());
            }
        }
        return diagnostics;
    }
}
