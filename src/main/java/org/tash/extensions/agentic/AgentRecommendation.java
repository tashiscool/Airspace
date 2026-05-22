package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentRecommendation {
    private String id;
    private String action;
    private String summary;
    private String rationale;
    private double confidence;
    private boolean humanApprovalRequired;
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
