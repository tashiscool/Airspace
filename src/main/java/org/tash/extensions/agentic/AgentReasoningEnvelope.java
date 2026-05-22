package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentReasoningEnvelope {
    private String id;
    private String promptVersion;
    private String modelId;
    private String reasoningMode;
    private String inputSummary;
    private String draftHash;
    private ZonedDateTime generatedAt;
    @Builder.Default
    private List<String> allowedFacts = new ArrayList<>();
    @Builder.Default
    private List<String> requiredOutputRules = new ArrayList<>();
    @Builder.Default
    private List<String> prohibitedActions = new ArrayList<>();
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
