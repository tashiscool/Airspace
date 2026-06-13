package org.tash.extensions.agentic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentAssessment {
    private String id;
    private String schemaVersion;
    private String agentType;
    private String claim;
    private String verdict;
    private double confidence;
    private String uncertainty;
    private String requiredHumanAction;
    private HumanReviewMode humanReviewMode;
    @Builder.Default
    private List<String> evidence = new ArrayList<>();
    @Builder.Default
    private List<String> counterEvidence = new ArrayList<>();
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
