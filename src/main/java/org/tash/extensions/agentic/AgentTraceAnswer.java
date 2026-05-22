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
public class AgentTraceAnswer {
    private String question;
    private String answer;
    private double confidence;
    @Builder.Default
    private List<String> ruleIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> unsupportedClaims = new ArrayList<>();
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
