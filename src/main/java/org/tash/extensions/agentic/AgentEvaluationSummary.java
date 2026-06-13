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
@Builder(toBuilder = true)
public class AgentEvaluationSummary {
    private boolean accepted;
    private int findingCount;
    private int recommendationCount;
    private int taskCount;
    private int deltaCount;
    private int citedClaimCount;
    private int uncitedClaimCount;
    private int policyViolationCount;
    private double citationCoverage;
    private boolean stabilityAccepted;
    @Builder.Default
    private Map<String, Integer> sourceFamilyCounts = new LinkedHashMap<>();
    @Builder.Default
    private List<String> stabilityWarnings = new ArrayList<>();
    @Builder.Default
    private List<String> warnings = new ArrayList<>();
    @Builder.Default
    private List<String> errors = new ArrayList<>();
}
